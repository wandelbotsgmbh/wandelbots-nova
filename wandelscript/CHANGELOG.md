# CHANGELOG


## v0.9.0 (2025-06-05)

### Features

- Updated nova version ([#31](https://github.com/wandelbotsgmbh/wandelscript/pull/31),
  [`94ce8d3`](https://github.com/wandelbotsgmbh/wandelscript/commit/94ce8d39df7d25037f4de9d125ae9254ca776f41))

Co-authored-by: Christoph Biering <christoph.biering@wandelbots.com>


## v0.8.1 (2025-05-27)

### Bug Fixes

- **runner**: Transform store to data_dict
  ([#30](https://github.com/wandelbotsgmbh/wandelscript/pull/30),
  [`b190e6e`](https://github.com/wandelbotsgmbh/wandelscript/commit/b190e6ec30bef4f37122dac39f8d78c701a4ea3b))

Co-authored-by: Christoph Biering <christoph.biering@wandelbots.com>


## v0.8.0 (2025-05-27)

### Features

- **runner**: Return result of runner
  ([#29](https://github.com/wandelbotsgmbh/wandelscript/pull/29),
  [`fca26c4`](https://github.com/wandelbotsgmbh/wandelscript/commit/fca26c40fb665d95e919706c7f4c5d509f0be10d))

Co-authored-by: Christoph Biering <christoph.biering@wandelbots.com>


## v0.7.0 (2025-05-26)

### Features

- **RPS-1615**: Implemented Nova program runner
  ([#28](https://github.com/wandelbotsgmbh/wandelscript/pull/28),
  [`2e6419c`](https://github.com/wandelbotsgmbh/wandelscript/commit/2e6419ce4c304ea3f12bd8e4d2ae742d5cb96342))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>

Co-authored-by: Dirk Sonnemann <dirk.sonnemann@wandelbots.com>


## v0.6.0 (2025-05-05)

### Features

- **RPS-1482**: Foreign functions with automatic type conversion; usable via CLI
  ([#18](https://github.com/wandelbotsgmbh/wandelscript/pull/18),
  [`7999674`](https://github.com/wandelbotsgmbh/wandelscript/commit/7999674970b2b1fc350c777e9da12893b013d731))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>

Co-authored-by: Dirk Sonnemann <dirk.sonnemann@wandelbots.com>


## v0.5.4 (2025-04-30)

### Bug Fixes

- **RPS-1543**: Rename argument in `runner.run()`
  ([`f2d5e96`](https://github.com/wandelbotsgmbh/wandelscript/commit/f2d5e96cedb893709d9ce55c7c9d1443784e3376))

...from `code` to `program`, in line with the new v2 openapi spec.


## v0.5.3 (2025-04-23)

### Chores

- Relax `typer` version constraints
  ([`cac77f1`](https://github.com/wandelbotsgmbh/wandelscript/commit/cac77f11ceac5f7f3c6ee587b50b8ebf9c410de3))

We want to use https://github.com/koxudaxi/fastapi-code-generator

to generate Python stubs from service-manager OpenAPI YAML endpoint definitions. Problem is that
  this tool requires `typer<0.13` to be installed.

Thus relax the constraints on `typer`.


## v0.5.2 (2025-04-15)

### Chores

- Remove an upper limit to the `nova` dependency
  ([`52eda71`](https://github.com/wandelbotsgmbh/wandelscript/commit/52eda71e1d277ee017e1853c3881cebfc5d86fc5))


## v0.5.1 (2025-04-14)

### Chores

- **RPS-1557**: Adjust files for use with `uv`
  ([`8932246`](https://github.com/wandelbotsgmbh/wandelscript/commit/89322469d17072893a5b355862c9aabfbe534e5e))

Notice especially in the `ci-dev.yaml` file, a certain change which I believe to fix something
  formerly incorrect:

- poetry install --without dev + pip install uv + uv sync --group dev

Evidently, the CI invokes `ruff` and other dev tools, thus, dev tools should explicitly be installed
  not ignored. The fact that this worked may be coincidental because other dependencies might have
  pulled ruff and the other tools in by default. However, let's better be explicit.

- **RPS-1557**: Migrate from `Poetry` to `uv`
  ([`263042d`](https://github.com/wandelbotsgmbh/wandelscript/commit/263042d282c5d58533ba8e71f3ec2442c455f68d))

# ymmv # install and run `migrate-to-uv` cd wandelscript poetry shell pip install migrate-to-uv uvx
  migrate-to-uv


## v0.5.0 (2025-04-07)

### Features

- **RPS-1438**: Improved runner models
  ([#23](https://github.com/wandelbotsgmbh/wandelscript/pull/23),
  [`14dbcbc`](https://github.com/wandelbotsgmbh/wandelscript/commit/14dbcbca0c100802827bf6b612f7c70c45f27e59))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>


## v0.4.1 (2025-04-04)

### Bug Fixes

- Cleanup serializer & fixed json types
  ([#22](https://github.com/wandelbotsgmbh/wandelscript/pull/22),
  [`51d546c`](https://github.com/wandelbotsgmbh/wandelscript/commit/51d546c6b29cb414772d75603e2efe3c2d5b7515))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>


## v0.4.0 (2025-04-04)

### Features

- **RPS-1438**: Simplified Wandelscript types
  ([#21](https://github.com/wandelbotsgmbh/wandelscript/pull/21),
  [`6f8d3f1`](https://github.com/wandelbotsgmbh/wandelscript/commit/6f8d3f162c326f4e2a8b772f7671f72343bc4fb4))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>


## v0.3.7 (2025-04-03)

### Bug Fixes

- **RPS-1490**: Avoid mutable type for apparent instance var
  ([`279d6fe`](https://github.com/wandelbotsgmbh/wandelscript/commit/279d6fe06f726c2da90de4f0a2df528323c61c27))

Looks like this is an instance var, not a class-level mutable type. Instead, initialize in
  `__init__()` function to avoid leaking of values across instances.


## v0.3.6 (2025-04-02)

### Bug Fixes

- **RPS-1490**: Make 2 motion classes dataclasses
  ([`583c70d`](https://github.com/wandelbotsgmbh/wandelscript/commit/583c70d6a2d74f8bddabc58b1f99e5432156a8dc))

Just like their siblings.

Also inherit one of them from `Connector.Impl` instead of from `Line`. Don't know why it inherited
  from Line tbh.


## v0.3.5 (2025-03-28)

### Chores

- Consolidate 2 strings into 1
  ([`f372fff`](https://github.com/wandelbotsgmbh/wandelscript/commit/f372fff0cef97fceebdc7cd1ce097e174a886cc0))

- Properly format an assertion
  ([`7ac3d67`](https://github.com/wandelbotsgmbh/wandelscript/commit/7ac3d676659906196112b98120d08ca24fc736b0))

Autoformatter goes brrr.

- Simplify Exception raises without params
  ([`3221862`](https://github.com/wandelbotsgmbh/wandelscript/commit/322186211266db57605b95cece0c8812aff9e887))

If no arguments are handed, they don't need parentheses.

- Turn an assertion around
  ([`3d8f0fc`](https://github.com/wandelbotsgmbh/wandelscript/commit/3d8f0fce58231566da25322d25ac8e4d76972eaa))

More canoncial.


## v0.3.4 (2025-03-28)

### Bug Fixes

- **RPS-1217**: Runner EStop handling
  ([`d1c8280`](https://github.com/wandelbotsgmbh/wandelscript/commit/d1c82801d03891c6d0ba2a68ab6e8fcf07dd4429))

- **rRPS-1217**: Fix dependencies & CI
  ([`12e1977`](https://github.com/wandelbotsgmbh/wandelscript/commit/12e197749fde5620668fe2dafc8aeed19066812b))

- **rRPS-1217**: Fix dependencies & CI
  ([`b0da731`](https://github.com/wandelbotsgmbh/wandelscript/commit/b0da73118b9b89137460f98ceb6b28a333f6b009))

- **rRPS-1217**: Fix dependencies & CI
  ([`e3c08af`](https://github.com/wandelbotsgmbh/wandelscript/commit/e3c08af7fb0329d1f74b3b0f65b30e5d89eebf44))

- **rRPS-1217**: Fix dependencies & CI
  ([`0870aa5`](https://github.com/wandelbotsgmbh/wandelscript/commit/0870aa54802bee631286ba78e03a4c933d881e85))

- **rRPS-1217**: Fix dependencies & CI
  ([`d6bea7d`](https://github.com/wandelbotsgmbh/wandelscript/commit/d6bea7da3eb455249e3c70540a7bb3399c953934))

- **rRPS-1217**: Fix dependencies & CI
  ([`3f7a4e9`](https://github.com/wandelbotsgmbh/wandelscript/commit/3f7a4e9f38ee645fb9b2f9f3b27f19b11574915a))

- **rRPS-1217**: Fix dependencies & CI
  ([`4eba6df`](https://github.com/wandelbotsgmbh/wandelscript/commit/4eba6df967502038e012cb5c259c055fc50dc825))

- **rRPS-1217**: Fix dependencies & CI
  ([`03afb30`](https://github.com/wandelbotsgmbh/wandelscript/commit/03afb30f678eb2e4524ebba49b5a53fd7a6fdc7f))


## v0.3.3 (2025-03-17)

### Chores

- **RPS-1310**: Replace `PTP` with `CartesianPTP`
  ([`3a49b91`](https://github.com/wandelbotsgmbh/wandelscript/commit/3a49b91f4b5df15e6809631e8fa280108e01a4d4))

Also require the newer version of nova that brings this change about.

- **RPS-1310**: Replace shortcut actions with verbose names
  ([`5974b5d`](https://github.com/wandelbotsgmbh/wandelscript/commit/5974b5deb8f4ef69f9a8cfe013ff7fb50ab5be24))


## v0.3.2 (2025-03-13)

### Bug Fixes

- **RPS-1309**: Read(robot, "pose") now considers the default_tcp
  ([#14](https://github.com/wandelbotsgmbh/wandelscript/pull/14),
  [`22445d2`](https://github.com/wandelbotsgmbh/wandelscript/commit/22445d2304fd94f7c0fab26efb92ce1cb06d5526))

Co-authored-by: Dirk Sonnemann <dirk.sonnemann@wandelbots.com>


## v0.3.1 (2025-03-12)

### Bug Fixes

- Allow for special characters in PR titles
  ([`2a8c8c2`](https://github.com/wandelbotsgmbh/wandelscript/commit/2a8c8c28154ec29991afeb52c9bf9f38a8d8b12b))

The prior way made backticks evaluate as bash or sh subshells commands.

E.g., I got following error:

Run PR_TITLE="feat(RPS-1311): Add pretty string repr for `PlanTrajectoryFailed` errors"
  /home/runner/work/_temp/7006b9fd-09b9-48b8-b2de-64f7a17ea6e7.sh: line 1: PlanTrajectoryFailed:
  command not found Error: Process completed with exit code 127.

Hereby forbid arbitrary code execution via PR Titles, as funny as that may be, and allow special
  characters such as backticks.

- Fix typos in `poses2.ws` example
  ([`d29e88e`](https://github.com/wandelbotsgmbh/wandelscript/commit/d29e88eecdc1c9014427b5fc0f3774131b4a1e02))

- **RPS-1311**: Improve error messages for `PlanTrajectoryErrors`
  ([`b4af166`](https://github.com/wandelbotsgmbh/wandelscript/commit/b4af166a6fd2d3e90b85abfe4406e824cde35b45))

In the case of PlanTrajectoryErrors, omit the very long output error lines in favor of a concise
  info about the error description.

This ALSO touches on the traceback, since it goes haywire in the CLI. Simple solution is to not set
  the ProgramRunner._exc variable. The output is so hyuge, thousands of lines of floating point
  numbers, nobody in their right mind would inspect them and hope to see something meaningful.

A somewhat technical but much lighter message is still conveyed to both robot pad and stdout.

Also require nova 0.47.0 in order to get access to the new pretty string method.

### Chores

- Improve output formatting in `cli.py`
  ([`af99e8e`](https://github.com/wandelbotsgmbh/wandelscript/commit/af99e8eee2f60c0fbad331f3ea7ba8a130bcfd16))


## v0.3.0 (2025-03-07)

### Chores

- Groom README.md
  ([`c20a504`](https://github.com/wandelbotsgmbh/wandelscript/commit/c20a50492f017c43984507266386e4c6a8d4f3fb))

- Remove a superfluous `pass`
  ([`c28ca44`](https://github.com/wandelbotsgmbh/wandelscript/commit/c28ca44d9a44d556955d7b2e0e3bd7de38905587))

- Remove an outdated TODO in `pyproject.toml`
  ([`94d0eec`](https://github.com/wandelbotsgmbh/wandelscript/commit/94d0eec4b87138a0c01eefc147883fe5299eadbd))

### Features

- **RPS-1286**: Add Wandelscript CLI executable
  ([`476cc10`](https://github.com/wandelbotsgmbh/wandelscript/commit/476cc107ee6867bb3caf45046e113b85a25bd52e))

Just a small one.

At the moment, only accesses a cell, everything else has to be defined inside the wandelscript.
  E.g., code like the following is supposed to work, assuming the cell and TCPs are set up prior:

tcp("flange") robot = get_controller("controller")[0] home = read(robot, "pose") sync

# Set the velocity of the robot to 200 mm/s velocity(200)

do with robot: for i = 0..3: move via ptp() to home move via line() to (0, 0, 0, 0, 0, 0) :: home
  move via line() to (200, 0, 0, 0, 0, 0) :: home move via ptp() to home

I see that in future we can specify cell, robot, TCPs and more via command line. However, baby
  steps. Thus adding this now hereby. I believe we can grow this over time and according to our
  usage patterns.

Install the CLI tool by calling:

poetry install

This installs the executable into your virtual environment.

You can then use the CLI via:

poetry run wandelscript --help poetry run wandelscript examples/my_script.cli poetry run ws
  examples/my_script.cli # shortcut alternative

I used `typer` for its apparent popularity within the team.

Note that importing wandelscript takes a horrendous amount of time, thus the CLI loads precariously
  slow.


## v0.2.2 (2025-03-07)

### Chores

- Update sdk version ([#12](https://github.com/wandelbotsgmbh/wandelscript/pull/12),
  [`d2f92bf`](https://github.com/wandelbotsgmbh/wandelscript/commit/d2f92bf2ffa39ce323196307e3c52c46b56ea198))


## v0.2.1 (2025-03-04)

### Chores

- Renamed flange to Flange to make it work in Wandelengine
  ([#10](https://github.com/wandelbotsgmbh/wandelscript/pull/10),
  [`479c911`](https://github.com/wandelbotsgmbh/wandelscript/commit/479c91113766f8a5f0cd2a4678020535daf2f8cd))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>


## v0.2.0 (2025-03-04)

### Features

- **RPS-1230**: Propagate motion recordings
  ([#9](https://github.com/wandelbotsgmbh/wandelscript/pull/9),
  [`ab9879f`](https://github.com/wandelbotsgmbh/wandelscript/commit/ab9879fd485319ce4709df256cad009636d19496))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>


## v0.1.0 (2025-02-26)

### Bug Fixes

- Check stdout on a run for a print message in a test
  ([`b4c6715`](https://github.com/wandelbotsgmbh/wandelscript/commit/b4c6715e7e4750a86e529b166fe05979d9a8d02f))

It's clear that the program's log contains "print something", since it contains `print("print
  something")`. However, upon looking closer, I realized that `runner.skill_run.logs` in fact does
  not contain the program's `print()` output, that's in 'runner.skill_run.stdout`.

Change the test accordingly.

- Fixed new nova execute interface & WS example
  ([#7](https://github.com/wandelbotsgmbh/wandelscript/pull/7),
  [`7fa497b`](https://github.com/wandelbotsgmbh/wandelscript/commit/7fa497bd92e77b7b0fd96625fae2ee383b0fa997))

---------

Co-authored-by: cbiering <christoph.biering@wandelbots.com>

- Remove unknown pytest options
  ([`1199ddf`](https://github.com/wandelbotsgmbh/wandelscript/commit/1199ddf607f0e164397ae423eee8e33f37b7f78e))

Extraenous ports over from the wandelbrain repo, can go away since their according packages are
  missing here.

- Typo
  ([`00312ce`](https://github.com/wandelbotsgmbh/wandelscript/commit/00312ce555d38d35f99764d34d77264db8761e98))

- **RPS-1006**: Fixed assoc does not work for tuple/array in Wandelscript
  ([#3](https://github.com/wandelbotsgmbh/wandelscript/pull/3),
  [`f264e17`](https://github.com/wandelbotsgmbh/wandelscript/commit/f264e17f1ba3bb9cd953c2b10fda84c334436a1e))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>

### Chores

- Add `darglint` to pre-commit
  ([`dfbdf2f`](https://github.com/wandelbotsgmbh/wandelscript/commit/dfbdf2fd80962b609b542590b52834b0d2efedfe))

- Add dependency `numpy`
  ([`8658369`](https://github.com/wandelbotsgmbh/wandelscript/commit/8658369f08bbc9834a51584aece920b7316e2a5f))

- Add pre-commit
  ([`a405d87`](https://github.com/wandelbotsgmbh/wandelscript/commit/a405d87d3cda118d287afe2ba617ae084d086217))

Disable pre-commit trailing whitespace checks on `whitespaces.ws`. We have ws-file that concerns
  itself with whitespaces and trailing whitespaces. Consequently, don't lint trailing whitespaces on
  this file.

Add hooks for yamllint. Also add a copy of the `.yamllint` config from the `wandelbots-nova` project
  to align on our formatting standards.

Add check for sorting imports.

Add mypy to pre-commit.

- Add test showcasing async foreign function
  ([`111fcd2`](https://github.com/wandelbotsgmbh/wandelscript/commit/111fcd27bb0f514a00de8dd2cbc221af606d4a04))

- Added basic CI
  ([`fc07f94`](https://github.com/wandelbotsgmbh/wandelscript/commit/fc07f94373ca18ec659bfaf0855c806fc28c2cd9))

- Align import style of dataclasses with `from dataclass ...`
  ([`164a20f`](https://github.com/wandelbotsgmbh/wandelscript/commit/164a20fa508d2817eec410addc99792b07ad46cb))

Hereby align the import style to the style apparent in the rest of the project.

- Extend `.gitignore`
  ([`436d0c8`](https://github.com/wandelbotsgmbh/wandelscript/commit/436d0c8b992f7c0e60524db7fac892ecafa590fd))

Ignore - vim swap files - some reports - .vscode/ folders - .python-version file

Ignore reports for: - pytest - mypy

- Extend the Readme
  ([`72bf6b8`](https://github.com/wandelbotsgmbh/wandelscript/commit/72bf6b8c51d1d03a8594c1cfd51b3248cf2e0994))

Give it a more welcoming feel.

- Format some code
  ([`8790a07`](https://github.com/wandelbotsgmbh/wandelscript/commit/8790a07d60e113e81ed7e4fb8bf700fb3f7ee825))

- Groom `generate_parser.sh`
  ([`6f2013b`](https://github.com/wandelbotsgmbh/wandelscript/commit/6f2013b1e3ef3c566cfac73875e7a2daec49910e))

`$OUTPUT_DIR` seems unused. I hereby remove it entirely.

- Groom the examples a bit
  ([`9dc9ae0`](https://github.com/wandelbotsgmbh/wandelscript/commit/9dc9ae0e0d6c486a50c4d4382b3b6cb6e938c731))

Especially reformat the json files for readability and a somewhat more canonical formatting.

It may not be perfect but I consider it better than before.

- Introduce internal _types
  ([`1dbd9ba`](https://github.com/wandelbotsgmbh/wandelscript/commit/1dbd9bacfeb6dec5f589ad51b7f6c2c8700c020e))

- Mention async funcitons in ForeignFunction docstrings
  ([`2885bcc`](https://github.com/wandelbotsgmbh/wandelscript/commit/2885bcc3d89ab122f3418fad0113c4de4c14b06d))

- Migrated from wandelengine repository
  ([`93cf4c5`](https://github.com/wandelbotsgmbh/wandelscript/commit/93cf4c5cedf45544a5a3a43253f40a5a84c18313))

- Remove `geometricalgebra` cont'd
  ([`db5f1eb`](https://github.com/wandelbotsgmbh/wandelscript/commit/db5f1eb6711677c4d270e5bca7328cd33e43dd14))

- Remove `geometricalgebra` Episode III
  ([`766fe95`](https://github.com/wandelbotsgmbh/wandelscript/commit/766fe95f0820ca1b9dd5c982538b8964ea829708))

Revenge of the euclideans.

- Remove a wrong entry from the README
  ([`0c5a976`](https://github.com/wandelbotsgmbh/wandelscript/commit/0c5a976d1c7a1f1b08ec612dc020437febcc5bb2))

- Remove an unused import in a test file
  ([`3a573b6`](https://github.com/wandelbotsgmbh/wandelscript/commit/3a573b68dbcd506bca943e566c34f92df585a339))

- Remove dependency `geometricalgebra`
  ([`4b1500f`](https://github.com/wandelbotsgmbh/wandelscript/commit/4b1500f262526c6b8c6f1a0346446a2b9a5c9ee0))

- Remove Orientation and Position
  ([`c213534`](https://github.com/wandelbotsgmbh/wandelscript/commit/c21353416430c18bc8fa90d1fab5b1a1f880fb39))

Now largely consolidated in nova's Vector3d.

p

- Remove plugins_addons.{py,ws}
  ([`671f4b5`](https://github.com/wandelbotsgmbh/wandelscript/commit/671f4b5ed2e7928501184b63e4ece6d9d3cc5e97))

- Remove serializer stuff
  ([`9c1cd5e`](https://github.com/wandelbotsgmbh/wandelscript/commit/9c1cd5e0c6d7f0086a3fd96246c108b74359b7dc))

Serialization is part of the classes and types that the Nova SDK brings, so away with it here.

- Remove some builtins
  ([`3ea0e81`](https://github.com/wandelbotsgmbh/wandelscript/commit/3ea0e818eb17ff7ab93782400f942e865ff2147f))

Some of the builtinins rather belong into the Nova SDK.

First step is to identify to see what should go over. Then, remove it here and then add it to the
  Nova SDK. Then implement it here.

For now, identify and delete here in order to get a slim, working barebones Wandelscript.

- Remove some trailing whitespaces
  ([`31c5dd1`](https://github.com/wandelbotsgmbh/wandelscript/commit/31c5dd19632d9b532db79d29929174e961f04ff9))

Came via:

poetry run pre-commit run --all

This fixed a few other whitespace issues in files that were either autogenerated or where trailing
  spaces were intentional. I did not commit those changes.

- Remove stray `pytest.xml` reports
  ([`56b68d2`](https://github.com/wandelbotsgmbh/wandelscript/commit/56b68d2d636e084c691f789fd8b2bfebf9dd9a0b))

Ciao cacao

- Remove unused `Spline` class
  ([`90cc557`](https://github.com/wandelbotsgmbh/wandelscript/commit/90cc55772c2a676fc08c80036b371f9d3d818114))

- Remove unused `tcp_pose()` builtin function
  ([`90d20a4`](https://github.com/wandelbotsgmbh/wandelscript/commit/90d20a45fad1ddd79cb0814e7f78ba7548371f8e))

- Run poetry lock
  ([`88b9135`](https://github.com/wandelbotsgmbh/wandelscript/commit/88b91357af634e7eb542dd2d60aa38860927bca8))

- Slim some docstrings and improve few type hints
  ([`aeda758`](https://github.com/wandelbotsgmbh/wandelscript/commit/aeda758571aa40673f1fa727e5bc6f68e1595ba0))

- Transition dts / pyjectory
  ([`18aa673`](https://github.com/wandelbotsgmbh/wandelscript/commit/18aa6738e7cc6e300f470c9f22a69f15839aff76))

- Updated deps & using wandelbots-nova package
  ([`8123185`](https://github.com/wandelbotsgmbh/wandelscript/commit/81231857e8e8092bcb9e384d6ee48562b1f303d6))

- Upgrade dependency `wandelbots-nova`
  ([`9c15556`](https://github.com/wandelbotsgmbh/wandelscript/commit/9c155560ddfee820ee0ac87a391284e7bdd86acb))

Also update the README a bit.

- Use backport for ExceptionGroups
  ([`d082eaf`](https://github.com/wandelbotsgmbh/wandelscript/commit/d082eaf6df7b9918f8d7283a543e47a1716f5910))

ExceptionGroups have their 1st party introduction with Python 3.11.

### Features

- Add a foreign function interface
  ([`7495698`](https://github.com/wandelbotsgmbh/wandelscript/commit/7495698f5536ac54d5b61b5d7b82219d94d9b216))

Pronounced Fiffi.

Change an according test to also assert the usage of ForeignFunctions.

- Added LICENSE
  ([`8c559cf`](https://github.com/wandelbotsgmbh/wandelscript/commit/8c559cfa12492ab32fb4687ce40bd562218d8e5f))

- **RPS-1266**: Added release CI to wandelscript
  ([`be1a4ad`](https://github.com/wandelbotsgmbh/wandelscript/commit/be1a4ade1a0fceceba7f58d8ccd2baa48451d1c0))

- **RPS-1267**: Renamed skill to program
  ([`f6aaebc`](https://github.com/wandelbotsgmbh/wandelscript/commit/f6aaebc9c81bd5be89d99b61d7ff716e99c61e68))

- **RPS-898**: Wandelscript migration ([#5](https://github.com/wandelbotsgmbh/wandelscript/pull/5),
  [`57b1ff0`](https://github.com/wandelbotsgmbh/wandelscript/commit/57b1ff0a3e2b90096ce7d54be11188ca4e029472))

Co-authored-by: cbiering <christoph.biering@wandelbots.com>
