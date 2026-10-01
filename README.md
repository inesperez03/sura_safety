# Diagnostics consumed by safety and BT

Safety consumes complete `DiagnosticArray` snapshots from `/diagnostics_agg`.
`global_diagnostics_aggregator` preserves each leaf's original diagnostic name
in `values[source_name]`. That source name is the machine identity; the published
`status.name` is a presentation path and can change without changing BT trees.

For example, source `/bluerov/Navigation/AreaLimit` can be displayed as
`/ROBOTS/BLUEROV/Navigation/AreaLimit` or under a different display root.
Both resolve from `Navigation/AreaLimit` when the monitor's robot namespace is
`bluerov`. Source identities are case-sensitive and retain the producer's case.

## Identity inputs

- A relative identity such as `Sensors/GPS` is scoped to the configured robot.
- An absolute source identity such as `/bluerov/Sensors/GPS` is explicit.
- An exact absolute published name is supported, but remains coupled to the
  display hierarchy. Prefer source identities in trees.
- No suffix matching, case folding, guessed display roots, or legacy label
  rewriting is performed. Ambiguous identities are unavailable.
- The subscription expects complete snapshots, not partial `/diagnostics`
  producer updates. Each new snapshot removes entries and aliases absent from it.

`DiagnosticsMonitor` owns resolution for both packages. Consumers must not
reconstruct aggregated display paths.

## Tree configuration

`bt_runner` derives each safety tree from `robot_namespace`: for example,
`robot_namespace:=bluerov` loads `trees/bluerov_safety_branch.xml`. Multiple
namespaces can be supplied as a comma-separated list; the runner loads one
safety tree per robot. With no namespace argument, it discovers robots from
fresh `/diagnostics_agg` source identities. A missing tree is a launch error,
so a robot cannot silently receive another robot's safety policy. Create a
matching file for each new robot namespace.

`SafetyError`, `SafetyCriticalError`, `SafetyErrorAsk`, and `SafetyWarning` require a `diagnostic`
input. The node's display `name` does not select a sensor. Error conditions use
`stale_is_error` (default `false`) to select whether producer STALE counts as an
error. Each robot tree explicitly defines its own per-sensor STALE policy.

`DiagnosticsUnavailableFor` takes comma-separated source identities, for example
`Sensors/ArucoPose,Sensors/GPS`. All entries must be missing, ERROR or STALE for
the configured `seconds` before it succeeds. An OK or WARN entry resets it.

`UpdateMissionControlFromSafety` receives `critical_diagnostics`,
`recoverable_diagnostics`, and `stale_error_diagnostics` lists in the XML.
`ComputeAreaRecoveryForce` requires `diagnostic_name="Navigation/AreaLimit"` and
reads its coordinates from a single fresh diagnostic snapshot.

Custom trees using implicit node-name selection, sensor-only names such as
`GPS`, or `diagnostic_suffix` must migrate to these explicit inputs. The bundled
safety tree is migrated.

Every BT leaf that commands or queries a robot requires an explicit
`robot_namespace` input. Trees pass the shared `{robot_namespace}` blackboard
value to that port. This includes controller and hardware leaves, motion
commands, navigation recovery, camera capture, and SURA action clients.

## Reception timeout

The ROS parameter `safety.diagnostics_timeout` specifies the maximum age of the
last received snapshot in seconds (default 5.0, finite and positive). A steady
clock detects a stopped aggregator even if ROS simulation time stops. Expired
entries are returned as STALE and become current again on the next snapshot.
Area recovery refuses missing or STALE entries.

The list of mandatory diagnostics, startup grace period and mission action on
missing mandatory data are not configured yet. The existing missing-data
semantics remain: error predicates do not treat a missing entry as an error;
`DiagnosticsUnavailableFor` does treat it as unavailable. These policy decisions
must be agreed before claiming that absent diagnostics prevent mission execution.

## Validation

The gtests exercise source identity lookup across display-root changes, isolation
between robots, ambiguous identities, snapshot removal, reception expiry and
recovery, explicit condition inputs, configured STALE policy, and GPS availability.
`sura_bt` also tests area recovery through the shared monitor.

Build and test `sura_safety` and `sura_bt` together so the executable and shared
library use the same API. Tests use intra-process ROS communication and do not
send robot commands. DDS transport across processes and actual robot behavior
still require an integration check in the target environment.

## SafetyErrorAsk

`Navigation/FrontObstacle` in the Bluegros and Bluerov trees uses `SafetyErrorAsk`.
An ERROR creates a pending decision and requests `mission_control=ask`; WARN and
STALE do not create a new decision. The pending decision remains in force even if
the diagnostic later clears. The runner must explicitly resolve it. After a
successful resume, the same uninterrupted ERROR is suppressed until the
producer reports a non-error status. An abort remains latched while ERROR
persists.
