# Contributing to KOMPAS

Thanks for your interest in KOMPAS. Bug reports, hardware compatibility
reports, documentation fixes and code are all welcome.

## Ways to help

- **Report a bug**: open an issue with steps to reproduce, what you expected,
  what happened, your OS and Python version, and the relevant part of `logs/`.
- **Report hardware compatibility**: tell us which drone you tried, whether it
  connected, and what did or didn't work. This is one of the most useful
  contributions right now.
- **Improve the docs**: anything that confused you will confuse the next person.
- **Write code**: for anything larger than a small fix, please open an issue
  first so we can agree on the approach before you put in the work.

## Development setup

```bash
git clone https://github.com/vishwagw/KOMPAS-autonomy.git
cd KOMPAS-autonomy
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt pytest
python -m pytest           # all tests run without hardware
python main.py --sim       # try your change in the simulator
```

## Pull requests

1. Fork the repo and create a branch from `main`.
2. Keep each PR focused on one change.
3. Add or update tests for behaviour you change. `tests/` has examples using
   mocks and the simulator.
4. Run `python -m pytest` and make sure everything passes.
5. Update the docs and `config/drone_config.yaml` comments if you add or
   change settings.
6. Describe what you changed and how you tested it in the PR.

## Safety-critical code

Code in `src/utils/safety_manager.py`, `src/utils/preflight.py`,
`src/communication/` and `src/control/` decides what a real aircraft does.
Changes there get extra review. In particular:

- Automatic actions must **land**, never cut motors.
- Never weaken a pre-flight check or safety limit by default.
- Test the change in simulator mode, including link loss and low battery
  (the **Cut link** and **Battery 15%** buttons), and say so in the PR.

## Scope

KOMPAS is for civilian and non-kinetic uses. Contributions that add
weaponization or targeting features will not be accepted.

## Code style

Follow the style of the surrounding code: PEP 8, type hints where they help,
docstrings on public classes and functions, and `logging` rather than `print`
in application code.

## License

By contributing, you agree that your contributions are licensed under the
[Apache License 2.0](LICENSE), the same license as the project.

## Conduct

Be respectful and constructive. See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
