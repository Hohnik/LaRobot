"""Export selection precedence and actual CLI callers use disposable recordings."""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from yam.recording import Trajectory
from yam.recording_slots import find_export_slot

ROOT = Path(__file__).resolve().parent.parent


def load_app(name):
    spec = importlib.util.spec_from_file_location('slot_test_' + name, ROOT / 'apps' / (name + '.py'))
    app = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(app)
    return app


def save(root, slot, marker, *, simulated=False, frames=False):
    folder = root / 'recordings' / 'sim' if simulated else root / 'recordings'
    folder.mkdir(parents=True, exist_ok=True)
    take = Trajectory(meta={'marker': marker, 'simulated': simulated})
    if frames:
        take.meta['cameras'] = {'per_camera': {'top': {'written': 1}}}
    take.append(0., [0.] * 14)
    take.append(.1, [.1] * 14)
    path = folder / f'{slot}.json'
    take.save(path)
    return path


def run(app, root, args):
    seen, output = [], io.StringIO()
    def export(take, **kwargs):
        seen.append((take.meta, kwargs))
        return SimpleNamespace(path=kwargs.get('out_path', root / 'recordings/datasets/train/episode'),
                               ticks=4, steps=4, duration_s=.1, mapping={'left': 'B', 'right': 'G'},
                               bad_spans=0, cameras={}, roles=(), view_size=(224, 224),
                               frame_size=(224, 224), warnings=[])
    function = 'export_dataset' if app.__name__.endswith('dataset') else 'export_episode'
    with patch.object(app, 'REPO', root), patch.object(app, function, export), \
            patch.object(sys, 'argv', ['export', '--left', 'B', '--right', 'G', *args]), \
            redirect_stdout(output):
        try:
            code = app.main()
        except SystemExit as exc:
            code = exc.code
    return code, output.getvalue(), seen


def test_resolver_prefers_real_and_warns_only_when_ambiguous():
    with TemporaryDirectory() as d:
        root = Path(d)
        real = save(root, '4', 'real')
        lines = []
        assert find_export_slot(root / 'recordings', '4', emit=lines.append) == real
        assert not lines
        save(root, '4', 'sim', simulated=True)
        assert find_export_slot(root / 'recordings', '4', emit=lines.append) == real
        assert lines == ['  ⚠️ slot 4 exists both real and simulated — exporting the REAL one.']


def test_resolver_fallback_warning_and_missing_error():
    with TemporaryDirectory() as d:
        root = Path(d)
        sim = save(root, '4', 'sim', simulated=True)
        lines = []
        assert find_export_slot(root / 'recordings', '4', emit=lines.append) == sim
        assert 'SIMULATED recording' in lines[0] and 'never for training' in lines[0]
        try:
            find_export_slot(root / 'recordings', '5')
        except FileNotFoundError as exc:
            assert str(exc) == '⛔ nothing saved in slot 5 (checked recordings/ and recordings/sim/).'
        else:
            raise AssertionError('missing slot accepted')


def test_both_actual_apps_use_their_own_recording_root_and_real_precedence():
    for name in ('export_episode', 'export_dataset'):
        with TemporaryDirectory() as d:
            root = Path(d)
            real = save(root, '4', 'real')
            save(root, '4', 'sim', simulated=True)
            app = load_app(name)
            code, text, seen = run(app, root, ['--slot', '4'])
            assert code == 0, text
            assert len(seen) == 1 and seen[0][0]['marker'] == 'real'
            assert seen[0][1]['recording_path'] == real
            assert 'exporting the REAL one' in text


def test_both_actual_apps_preserve_simulation_metadata():
    for name in ('export_episode', 'export_dataset'):
        with TemporaryDirectory() as d:
            root = Path(d)
            sim = save(root, '4', 'sim', simulated=True)
            code, text, seen = run(load_app(name), root, ['--slot', '4'])
            assert code == 0, text
            assert seen[0][0]['simulated'] is True and seen[0][1]['recording_path'] == sim
            assert 'never for training' in text


def test_both_actual_apps_refuse_missing_slots_before_export():
    for name in ('export_episode', 'export_dataset'):
        with TemporaryDirectory() as d:
            code, _, seen = run(load_app(name), Path(d), ['--slot', '9'])
            assert code == '⛔ nothing saved in slot 9 (checked recordings/ and recordings/sim/).'
            assert not seen


def test_batch_never_substitutes_framed_sim_for_frameless_real():
    with TemporaryDirectory() as d:
        root = Path(d)
        save(root, '4', 'real', frames=False)
        save(root, '4', 'sim', simulated=True, frames=True)
        code, text, seen = run(load_app('export_dataset'), root, ['--all'])
        assert 'no recording carries camera frames' in code
        assert 'skipping 1 recording(s)' in text and not seen


def test_batch_uses_same_precedence_and_warns_once_per_export():
    with TemporaryDirectory() as d:
        root = Path(d)
        save(root, '4', 'real', frames=True)
        save(root, '4', 'shadowed', simulated=True, frames=True)
        save(root, '7', 'sim', simulated=True, frames=True)
        code, text, seen = run(load_app('export_dataset'), root, ['--all'])
        assert code == 0, text
        assert [meta['marker'] for meta, _ in seen] == ['real', 'sim']
        assert [args['episode_id'] for _, args in seen] == ['slot4', 'slot7']
        assert text.count('exporting the REAL one') == text.count('SIMULATED recording') == 1


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
