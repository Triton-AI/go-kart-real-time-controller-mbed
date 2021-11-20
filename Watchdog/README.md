# Watchdog

## Purpose

Every active component on the controller must be monitored by a watchdog instance. Since every component in our software architecture lives on a different thread, this conviently ensures that every thread of the application has a watchdog.

To define sets of behaviors that the watchdog should be expecting from the component being watched, an interface needs to be designed.

## Components

`watchdog.hpp` contains the `Watchdog` class.

`watchable.hpp` contains `Watchable` class which serves as the interface for an object to be wached by the watchdog.
## API

### `Watchdog`

```cpp
Watchdog(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms)
```

The watchdog should be initialized in a main controller which itself could be watchable. Since the watchdog itself is a `Watchable` object and will watch itself and reset if necessary, two params (`update_interval_ms` and `max_inactivity_limit_ms` are passed to initialize itself as a `Watchable` object).

```cpp
void add_to_watchlist(Watchable* to_watch)
```

After initialization, the watchdog must be added with a list of all watchable objects.

You could pass the main controller that initialized the watchdog to it as well.

```cpp
void arm()
```

Start the watchdog

```cpp
void disarm()
```

Pause the watchdog

## Inner-Working

## Known Issues and Future Improvements
