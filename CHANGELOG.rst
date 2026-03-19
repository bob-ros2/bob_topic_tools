^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bob_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-03-20)
------------------
* Comprehensive refactoring for ROS 2 compliance.
* Added environment variable support for default parameter values across all nodes.
* Updated `ParameterDescriptor` text to include environment variable references.
* `FilterNode`: Refactored `substitute` parameter to support multiple pattern-replace pairs.
* `AggregatorNode`: Fixed memory leak and performance issues by switching to a single persistent timer.
* `AggregatorNode`: Corrected typo `delimeter` to `delimiter` in parameters and logging.
* `TerminalNode`: Shortened long lines and addressed linter warnings.
* Full PEP 8 and PEP 257 compliance across all Python files.
* Overhauled `README.md` with table-based API documentation.
* Added missing test dependencies and metadata to `package.xml`.
* Enabled automatic linting via `ament_lint_auto`.

1.0.1 (2024-10-08)
------------------
* Initial release of bob_topic_tools
* Contributors: Bob Ros
