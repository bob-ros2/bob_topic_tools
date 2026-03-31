^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bob_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-03-20)
------------------
* `FilterNode`: Enhanced with `trim_data`, `trim_chars`, and `skip_empty` for robust message cleaning.
* `TerminalNode`: Modernized parameter management with runtime update support.
* `AggregatorNode`: Added support for multi-character delimiters (e.g., `. `) and transitioned `delimiters` to a string array.
* `DrainNode`: Renamed from `ValveNode` to better reflect its "draining" behavior. Standardized topics to `drain_in` and `drain_out`.
* Refactored nodes to prioritize native ROS 2 parameters and removed redundant environment variable overrides.
* Full PEP 8 and PEP 257 compliance across all Python files.
* Enabled automatic linting via `ament_lint_auto`.

1.0.1 (2024-10-08)
------------------
* Initial release of bob_topic_tools
* Contributors: Bob Ros
