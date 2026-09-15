"""A failed camera cleanup must not leave the other camera threads running."""
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
import sys
from yam.cameras.capture import CaptureSet
from yam.cameras.grabber import FrameGrabber
from yam.cameras.writer import FrameSink


def test_all_readers_stopped_after_one_failure():
    events = []
    def bad():
        events.append('B')
        raise OSError('reader failure')
    captures = CaptureSet({'B': SimpleNamespace(stop=bad),
                           'G': SimpleNamespace(stop=lambda: events.append('G'))})
    try:
        captures.stop()
    except ExceptionGroup as exc:
        assert len(exc.exceptions) == 1
    else:
        raise AssertionError('cleanup failure hidden')
    assert events == ['B', 'G']


def test_all_writers_stopped_after_one_failure():
    events = []
    def bad():
        events.append('B')
        raise OSError('index write failed')
    sink = FrameSink.__new__(FrameSink)
    sink._writers = {'B': SimpleNamespace(stop=bad),
                     'G': SimpleNamespace(stop=lambda: events.append('G'))}
    try:
        sink.stop()
    except ExceptionGroup as exc:
        assert 'index write failed' in str(exc.exceptions[0])
    else:
        raise AssertionError('cleanup failure hidden')
    assert events == ['B', 'G']


def test_partial_writer_startup_closes_previous_writer():
    events = []
    def writer(name, *args):
        if name == 'G':
            raise OSError('directory creation failed')
        return SimpleNamespace(stop=lambda: events.append(name))
    with patch('yam.cameras.writer.FrameWriter', writer):
        try:
            FrameSink(Path('/unused'), ['B', 'G'])
        except OSError as exc:
            assert str(exc) == 'directory creation failed'
        else:
            raise AssertionError('startup should fail')
    assert events == ['B']


def test_grabber_releases_device_when_join_fails():
    events = []
    def join(**kwargs):
        raise RuntimeError('join failure')
    grabber = FrameGrabber.__new__(FrameGrabber)
    grabber._thread = SimpleNamespace(join=join)
    grabber._cap = SimpleNamespace(release=lambda: events.append('release'))
    try:
        grabber.stop()
    except RuntimeError:
        pass
    else:
        raise AssertionError('join failure hidden')
    assert events == ['release'] and grabber._running is False


def main():
    tests = [v for k, v in globals().items() if k.startswith('test_') and callable(v)]
    passed = 0
    for test in tests:
        try:
            test()
            passed += 1
            print('✓', test.__name__)
        except Exception as exc:
            print('✗', test.__name__, repr(exc))
    print(f'{passed}/{len(tests)} passed')
    return int(passed != len(tests))


if __name__ == '__main__':
    sys.exit(main())
