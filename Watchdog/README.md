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
Watchdog(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms, uint32_t wakeup_every_ms)
```

The watchdog should be initialized in a main controller which itself could be watchable. Since the watchdog itself is a `Watchable` object and will watch itself and reset if necessary, the first two params (`update_interval_ms` and `max_inactivity_limit_ms`) are passed to initialize itself as a `Watchable` object. 

The last param `wakeup_every_ms` sets the internal frequency of the watchdog, which ideally should be ten times smaller than the minimum update interval of any `Watchable` object to be added.

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

### `Watchable`

To enable watchdog function in a class, it needs to subclass `Watchable` and implement `watchdog_callback()`.

```cpp
Watchable(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms)
```

A watchable object must be initialized with two params:

- `update_interval_ms`: the promised update interval of activity, and
- `max_inactivity_limit_ms`: the max duration that the watchdog will tolerate before triggering.

```cpp
void activate()
```

Let the watchdog start to monitor this object.

```cpp
void deactivate()
```

Let the watchdog stop monitoring this object.

```cpp
void inc_count()
```

Call this function to reset watchdog countdown. Under the hood there is a rolling counter signaling activity.

## Inner-Working

## Known Issues and Future Improvements
