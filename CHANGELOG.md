# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
## 0.3.9 - 2022-03-21
### Added
- Readme file.
- Link to the new repository.
- Python dependencies.

### Changed
- Updated package manifest format to version 3.
- Maintainer is not set to the community email.

### Deprecated
- Starting from v0.4.0 this package will be available only on GitHub. This is the last version here.

## 0.3.1 - 2021-06-28
### Added
- Parameter `reload_map` to block / allow usage of newer maps.

## 0.3.0 - 2021-03-16
### Added
- Show inflation progress in the terminal.
- Use map rotation when generating the cells.
- Add callback for external source of simple trajectory points via `Path`.

### Changed
- Set default values inflation parameters to 0.
- Increase maximum values of inflation parameters from 40 to 100.

### Fixed
- Cell size is same as size of a map cell.
- Marker for 'clicked_point' is using proper `frame_id`.
- Publish inflated map in the callback as well.

## 0.2.1 - 2020-09-18
### Added
- Add commentary along with usage manual to the node.

## 0.2.0 - 2020-09-18
### Added
- Path creation is now interactive -- control points can be added, moved, and deleted from the path.
- Global map is now received and used to generate valid cells in the area of the path.
- Inflation rate (for map and for the path) can be modified using dynamic reconfigure.
- Add launch file.

### Changed
- Node is renamed from `simple_trajectory.py` to `node.py` to avoid naming conflicions.

## 0.1.1 - 2020-02-26
### Changed
- Publishers are now latched, messages are no longer periodically resent.
- Selected points are printed out in a Python-friendly format (can be copy-pasted right to the code).

## 0.1.0 - 2019-12-11
### Added
- First version of simple trajectory generator.
- Add publisher for the trajectory as a `Marker`.