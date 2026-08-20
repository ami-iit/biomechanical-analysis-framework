# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- The capability to provide "ik-solution-link-orientation" and "target-imu-orientation" as quantities (https://github.com/ami-iit/component_human-biomechanics/issues/47)
- The HumanIK class (https://github.com/ami-iit/biomechanical-analysis-framework/pull/15)
- The `CHANGELOG.md` file
- The `Logging` feature (https://github.com/ami-iit/biomechanical-analysis-framework/pull/10)
- `FloorContactTask` config parameter `default_position`: per-component (x,y,z) override used when no real contact position is known yet.
- `FloorContactTask` config parameter `contact_position`: per-component (x,y,z) override applied when contact becomes active; default is `["*", "*", "0.0"]`.

### Removed
- The `linkHeight` argument of `HumanIK::updateFloorContactTask()` and `HumanIK::updateFloorContactTasks()` (and the corresponding Python bindings argument), replaced by the per-task `default_position` config.

