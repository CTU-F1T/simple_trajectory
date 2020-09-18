# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
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