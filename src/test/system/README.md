# Testing

To launch the system test execute:

```bash
launch_test <path to system test python file>
```

To launch the IMU system text script:
```bash
pytest -sq --tb=short <path to python test file>
```

Maybe you have to use pytest-3:
```bash
pytest-3 -sq --tb=short <path to python test file>
```
