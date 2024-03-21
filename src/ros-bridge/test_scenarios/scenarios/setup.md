Run judge, stop-continue

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="judge-judge.json" labels:="['judge', 'judge']" n:=1 record_gaze:=True 
```

Run kid -> terrorist, stop-continue

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="kid-terrorist.json" labels:="['police', 'terrorist']" n:=2 record_gaze:=True 
```

Run terrorist -> kid, stop-continue

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="judge-judge.json" labels:="['judge', 'judge']" n:=3 record_gaze:=True 
```

Run judge pass

```bash
ros2 launch test_scenarios test_scenarios.launch.py config:="judge-judge(cross).json" labels:="['judge', 'judge']" n:=4 record_gaze:=True 
```
