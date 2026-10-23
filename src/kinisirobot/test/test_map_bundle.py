"""Tests for portable Kinisi map bundles."""

import io
import json
import sys
import zipfile
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from kinisirobot.map_bundle import (  # noqa: E402
    MapBundleError,
    create_map_bundle,
    install_map_bundle,
    safe_map_name,
)


def write_map_files(base, include_posegraph=True):
    base.with_suffix(".yaml").write_text(
        f"image: {base.name}.pgm\n", encoding="utf-8"
    )
    base.with_suffix(".pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    if include_posegraph:
        base.with_suffix(".posegraph").write_bytes(b"posegraph")
        base.with_suffix(".data").write_bytes(b"data")


def make_zip(files):
    output = io.BytesIO()
    with zipfile.ZipFile(output, "w") as archive:
        for name, data in files.items():
            archive.writestr(name, data)
    return output.getvalue()


def test_bundle_round_trip_includes_posegraph(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    base = source / "office"
    write_map_files(base)
    bundle = tmp_path / "office.kinisi-map.zip"

    manifest = create_map_bundle(str(base), "office", str(bundle))
    result = install_map_bundle(bundle.read_bytes(), str(tmp_path / "maps"))

    assert manifest["posegraph"] is True
    assert result["posegraph"] is True
    assert Path(result["base"] + ".yaml").is_file()
    assert Path(result["base"] + ".data").read_bytes() == b"data"


def test_partial_install_restores_previous_complete_map(tmp_path, monkeypatch):
    """A filesystem failure must not leave a mixed old/new map snapshot."""
    from kinisirobot import map_bundle
    source = tmp_path / 'source'
    destination = tmp_path / 'maps'
    source.mkdir()
    destination.mkdir()
    write_map_files(source / 'office')
    archive = tmp_path / 'map.zip'
    create_map_bundle(str(source / 'office'), 'office', str(archive))
    for extension in ('.yaml', '.pgm', '.posegraph', '.data'):
        (destination / ('office' + extension)).write_bytes(b'previous')
    replace = map_bundle.os.replace
    def fail_second(source_path, destination_path):
        if str(destination_path).endswith('office.pgm') and not str(source_path).endswith('.previous'):
            raise OSError('simulated storage failure')
        return replace(source_path, destination_path)
    monkeypatch.setattr(map_bundle.os, 'replace', fail_second)
    with pytest.raises(OSError):
        install_map_bundle(archive.read_bytes(), str(destination), require_posegraph=True)
    for extension in ('.yaml', '.pgm', '.posegraph', '.data'):
        assert (destination / ('office' + extension)).read_bytes() == b'previous'


def test_bundle_round_trip_allows_occupancy_only(tmp_path):
    base = tmp_path / "static"
    write_map_files(base, include_posegraph=False)
    bundle = tmp_path / "static.kinisi-map.zip"

    create_map_bundle(str(base), "static", str(bundle))
    result = install_map_bundle(bundle.read_bytes(), str(tmp_path / "maps"))

    assert result["occupancy"] is True
    assert result["posegraph"] is False


def test_required_posegraph_is_checked_before_install(tmp_path):
    base = tmp_path / "static"
    write_map_files(base, include_posegraph=False)
    bundle = tmp_path / "static.kinisi-map.zip"
    maps_dir = tmp_path / "maps"
    create_map_bundle(str(base), "static", str(bundle))

    with pytest.raises(MapBundleError, match="no SLAM pose graph"):
        install_map_bundle(
            bundle.read_bytes(), str(maps_dir), require_posegraph=True
        )

    assert not maps_dir.exists()


@pytest.mark.parametrize("name", ["../map", "map/name", "map name", ""])
def test_unsafe_map_names_are_rejected(name):
    with pytest.raises(MapBundleError):
        safe_map_name(name)


def test_archive_path_traversal_is_rejected(tmp_path):
    manifest = json.dumps(
        {"format": "kinisi-map", "version": 1, "name": "office"}
    )
    payload = make_zip(
        {
            "manifest.json": manifest,
            "office.yaml": "image: office.pgm\n",
            "office.pgm": b"pgm",
            "../escape.data": b"bad",
        }
    )

    with pytest.raises(MapBundleError, match="unsafe path"):
        install_map_bundle(payload, str(tmp_path / "maps"))


def test_unexpected_archive_members_are_rejected(tmp_path):
    manifest = json.dumps(
        {"format": "kinisi-map", "version": 1, "name": "office"}
    )
    payload = make_zip(
        {
            "manifest.json": manifest,
            "office.yaml": "image: office.pgm\n",
            "office.pgm": b"pgm",
            "notes.txt": b"unexpected",
        }
    )

    with pytest.raises(MapBundleError, match="unexpected files"):
        install_map_bundle(payload, str(tmp_path / "maps"))
