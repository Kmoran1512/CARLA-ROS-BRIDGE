Run judge, stop-continue

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="judge-judge.json" labels:="['judge', 'judge']" n:=1
```

Run kid -> terrorist, stop-continue

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="kid-terrorist.json" labels:="['kid', 'terrorist']" n:=2
```

Run terrorist -> kid, stop-continue

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="judge-judge.json" labels:="['judge', 'judge']" n:=3
```

Run judge pass

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="judge-judge(cross).json" labels:="['judge', 'judge']" n:=4
```
