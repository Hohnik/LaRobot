"""Real writer threads and blocked I/O exercise take ownership without hardware."""
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
import sys
from unittest.mock import patch
from yam.cameras.frame import Frame
from yam.cameras.writer import FrameSink, FrameWriter
from yam.recording_session import RecordingSession
from yam.recording_store import save_take


def eventually(check, timeout=2.):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if check():
            return
        time.sleep(.005)
    raise AssertionError('completion did not arrive')


def stopped_take(root, encode=lambda rgb: b'image'):
    take = RecordingSession()
    take.start(1., meta={'simulated': True}, modes=['B:hold'])
    take.sample(1., [0.] * 7, [])
    take.sample(2., [.1] * 7, [])
    take.start_frames(root / 'frames/pending_test', ['camera'], 123,
                      factory=lambda path, names: FrameSink(path, names, encode=encode))
    take.offer_frames({'camera': Frame('camera', 1, None, 124, object())})
    take.freeze()
    return take


def poll_until(take, check):
    def step():
        take.poll()
        return check()
    eventually(step)


def test_blocked_writer_never_publishes_a_partial_index():
    release, entered = threading.Event(), threading.Event()
    def encode(rgb):
        entered.set()
        assert release.wait(2.)
        return b'image'
    with TemporaryDirectory() as d:
        writer = FrameWriter('camera', Path(d), encode=encode)
        try:
            writer.offer(Frame('camera', 1, None, 124, object()))
            assert entered.wait(1.)
            assert writer.stop(timeout=0)['flushed'] is False
            assert writer.poll_stop() is None
            assert not (Path(d) / 'index.json').exists()
            try:
                writer.offer(Frame('camera', 2, None, 125, object()))
            except RuntimeError:
                pass
            else:
                raise AssertionError('writer accepted a frame after stop')
        finally:
            release.set()
            report = writer.stop()
        assert report['flushed'] and report['written'] == 1
        assert (Path(d) / 'index.json').exists()


def test_discard_waits_for_encoder_and_then_deletes_off_thread():
    release, entered = threading.Event(), threading.Event()
    def encode(rgb):
        entered.set()
        assert release.wait(2.)
        return b'image'
    with TemporaryDirectory() as d:
        take = stopped_take(Path(d), encode)
        frames = take.frames
        try:
            assert entered.wait(1.)
            take.request_discard()
            for _ in range(30):
                take.poll()
            assert take.busy and frames.exists() and take.pending is not None
            try:
                take.request_save(Path(d), '1')
            except RuntimeError:
                pass
            else:
                raise AssertionError('save allowed while encoder still writes')
            assert not (Path(d) / '1.json').exists()
        finally:
            release.set()
            poll_until(take, lambda: not take.busy)
        assert take.pending is None and not frames.exists()


def test_shutdown_retains_unfinished_writer_and_directory():
    release, entered = threading.Event(), threading.Event()
    def encode(rgb):
        entered.set()
        assert release.wait(2.)
        return b'image'
    with TemporaryDirectory() as d:
        take = stopped_take(Path(d), encode)
        try:
            assert entered.wait(1.)
            try:
                take.shutdown(timeout=0)
            except RuntimeError as exc:
                assert 'retained' in str(exc)
            else:
                raise AssertionError('incomplete shutdown reported success')
            assert take.sink is not None and take.frames.exists()
        finally:
            release.set()
            poll_until(take, lambda: take.ready)
            take.request_discard()
            poll_until(take, lambda: not take.busy)


def test_index_failure_retains_take_and_can_be_discarded_after_termination():
    write_text = Path.write_text
    def broken(path, *args, **kwargs):
        if path.name == 'index.json':
            raise OSError('injected index failure')
        return write_text(path, *args, **kwargs)
    with TemporaryDirectory() as d, patch.object(Path, 'write_text', broken):
        take = stopped_take(Path(d))
        poll_until(take, lambda: take.sink is None)
        assert 'injected index failure' in take.frame_error
        assert take.frames.exists() and take.pending is not None and not take.ready
        try:
            take.request_save(Path(d), '1')
        except RuntimeError:
            pass
        else:
            raise AssertionError('failed camera take was publishable')
        take.request_discard()
        poll_until(take, lambda: not take.busy)
        assert take.frames is None


def test_blocked_save_keeps_ownership_and_failure_allows_same_take_retry():
    release, entered = threading.Event(), threading.Event()
    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(2.)
        raise OSError('disk failed')
    with TemporaryDirectory() as d:
        root = Path(d)
        take = stopped_take(root)
        poll_until(take, lambda: take.ready)
        pending, frames = take.pending, take.frames
        try:
            take.request_save(root, '1', blocked)
            assert entered.wait(1.)
            for _ in range(30):
                take.poll()
            assert take.busy and take.pending is pending and frames.exists()
            try:
                take.request_discard()
            except RuntimeError:
                pass
            else:
                raise AssertionError('discard raced slot publication')
        finally:
            release.set()
            poll_until(take, lambda: not take.busy)
        assert take.pending is pending and take.frames == frames and take.save_error
        take.request_save(root, '2', save_take)
        poll_until(take, lambda: not take.busy)
        assert take.saved.path == root / '2.json' and take.pending is None
        assert (root / 'frames/2/camera/index.json').exists()


def test_failed_delete_keeps_files_and_can_retry():
    with TemporaryDirectory() as d:
        take = stopped_take(Path(d))
        poll_until(take, lambda: take.ready)
        frames = take.frames
        with patch('yam.recording_session.discard_frames', side_effect=OSError('delete failed')):
            take.request_discard()
            poll_until(take, lambda: not take.busy)
        assert take.pending is not None and frames.exists() and take.save_error
        take.request_discard()
        poll_until(take, lambda: not take.busy)
        assert not frames.exists() and take.pending is None


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
