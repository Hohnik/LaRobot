"""Camera discovery, acquisition, sampling and per-take writing.

- discovery resolves device names and probes OpenCV indices without list-order guesses.
- identity reads USB serials and Linux by-id metadata.
- open owns shared capture configuration and frame-delivery measurement.
- session assembles recording readers under partial-startup ownership.
- grabber buffers one device; capture samples named readers without waiting.
- writer drains per-take queues and publishes final indexes after completion.

Applications consume this library. Importing it never enumerates or opens devices.
See docs/CLEANUP.md for current validation and remaining hardware limitations.
"""
