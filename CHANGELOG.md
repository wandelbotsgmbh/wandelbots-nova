# CHANGELOG


## v0.25.0 (2025-02-10)

### Features

* feat: only plan and not execute in welding example (#57) ([`3155e6f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3155e6fb7fd3623bee7ea9848465265761780897))


## v0.24.1 (2025-02-10)

### Fixes

* fix: simple check for http prefix (#42)

Co-authored-by: André <andre.kuehnert@wandelbots.com> ([`cd4b478`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/cd4b478bb551f7a444ef10f696980d4a2a059067))


## v0.24.0 (2025-02-10)

### Features

* feat: add optional rerun bridge (#45) ([`00177e9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/00177e95a056bca997f5e2cee6f59d6b1e0b3f00))


## v0.23.0 (2025-02-10)

### Features

* feat: enhance trajectory planning with optional starting joint position (#49) ([`2eeb747`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2eeb747bfcac6da28a1ea629396eb89c1c246cc3))


## v0.22.0 (2025-02-10)

### Features

* feat: added __eq__ to vector and pose (#48)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`8198eb0`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8198eb08edf0eda150bed202fc619ec42c7c3e99))


## v0.21.0 (2025-02-07)

### Features

* feat: streaming execution first draft (#43)

Co-authored-by: Dirk Sonnemann <dirk.sonnemann@wandelbots.com> ([`11bb47a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/11bb47a94e4370c69291af59eb8b425e693a7c5d))


## v0.20.0 (2025-02-07)

### Features

* feat: propagate robot state in motion group (#47)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`73e87d2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/73e87d24cc6ebede177a074cdbf5ef76a956b9dd))


## v0.19.1 (2025-02-07)

### Chores

* chore: added active tcp to abstract robot (#46)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`f64cb39`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f64cb393164cb92ae8ec5173ca32ce912c390420))


## v0.19.0 (2025-02-07)

### Features

* feat: get active_tcp_name & added motion_settings to var name (#44)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`352b17b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/352b17b5ee63e3e747b7ef19f85226b644db3b07))


## v0.18.0 (2025-02-06)

### Features

* feat(RPS-1206): implemented I/O access, read & write (#41)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`25e6caa`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/25e6caace690d607a1ea44eb72ab521e8bc35da8))


## v0.17.5 (2025-02-06)

### Chores

* chore: make robot cell devices readable (#40)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`b7bbbbf`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b7bbbbf9b2db6a0d76c99acd232a7bdeb61180d3))


## v0.17.4 (2025-02-05)

### Chores

* chore: enable example tests in CI (#39)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`929db4e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/929db4e2b8d71137b4bf2f5dd20c9248ca661e30))


## v0.17.3 (2025-02-05)

### Chores

* chore(RPS-898): implemented robotcell + execute * abstract robot and deps (#38)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`45eec6a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/45eec6aad6fb5576210176e2eab1d0d062e33fff))


## v0.17.2 (2025-02-05)

### Chores

* chore: update wandelbots_api_client to version 25.1.0 (#37) ([`ec3fd3d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/ec3fd3d4482c900915f0a32f3cb763529d585618))


## v0.17.1 (2025-02-04)

### Fixes

* fix: exception naming (#36)

Co-authored-by: Dirk Sonnemann <dirk.sonnemann@wandelbots.com> ([`723b2d8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/723b2d8fff88a32c44003b6e3bbf4499396f7890))


## v0.17.0 (2025-02-04)

### Features

* feat: Make authorization flow asynchron. (#34)

Co-authored-by: mahsumdemir <mahsum.demir@wandelbots.com> ([`2e0dae6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2e0dae6b1c2ddb56ff66cbcf4a3033165a414a55))


## v0.16.1 (2025-02-04)

### Fixes

* fix(RPS-1174): dont deactivate motion groups (#35) ([`67b6535`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/67b6535a3f519b74807ab54db0df4e48458a06a5))


## v0.16.0 (2025-01-23)

### Features

* feat: expose error object in exception (#32) ([`0b7351f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0b7351f3520bf96e5331c1693c7e6dd2c8d477da))


## v0.15.1 (2025-01-22)

### Chores

* chore: improved example descriptions (#33)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`c73faa5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c73faa5669de8e1d98c6057ebee40783749cb83c))


## v0.15.0 (2025-01-22)

### Features

* feat(RPS-976): added integration test CI (#25)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`15fc353`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/15fc35340c485ba6ceda241039bff5a468018b24))


## v0.14.0 (2025-01-20)

### Features

* feat: Add Auth0 device code authorization and refresh token flow (#31)

Co-authored-by: Christoph Biering <1353438+biering@users.noreply.github.com> ([`06f42ab`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/06f42ab2f22511ad7c8ed6fc4ad673008b9f49e9))


## v0.13.0 (2025-01-20)

### Features

* feat: expose virtual robot api (#30) ([`a43f73a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a43f73aa055aae2811e93d9f5dff6a8f9c8511f1))


## v0.12.0 (2025-01-16)

### Features

* feat: expose store collision scene api (#28) ([`d92b4dc`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d92b4dcda1e56246230d2ad99fb502cb3da755f2))


## v0.11.0 (2025-01-16)

### Features

* feat: add support for motion setting (#23) ([`dbbe8a6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/dbbe8a637608b5ef2e649739c353b63a84169ebc))


## v0.10.1 (2025-01-15)

### Fixes

* fix(RPS-1086): add option to close the nova object (#27)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`e30eabc`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e30eabc9fb35e07e9def4fc52e29b1e3aa60ed24))


## v0.10.0 (2025-01-15)

### Features

* feat: expose store collision components api (#26) ([`0bfd51e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0bfd51e5476b7c91d456380bc6d318c30caf50e7))


## v0.9.1 (2025-01-14)

### Chores

* chore: updated readme (#24)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`fe17057`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/fe170573ecfabb06df46fae11ebf7905af7f9b0e))


## v0.9.0 (2025-01-13)

### Chores

* chore: Add pre-commit hook to sort imports

...with ruff. ([`1a190f5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1a190f5ed87d05082f883d081645017af3ede72f))

* chore: Add yamllint to pre-commit

CI reported yaml issues when I changed the `.pre-commit-config.yaml` in
the last commit. Thus, I decided to also add yamllint as a check for
pre-commit, so next time I get quicker feedback.

See:
https://yamllint.readthedocs.io/en/stable/integration.html ([`aa552fe`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/aa552fe121f81d45674be13903f61b54c782010d))

* chore: Add mypy checks to pre-commit

Not thaaat fast, but quicker than CI.

Because pre-commit runs mypy from an isolated virtualenv (without our
dependencies), a run to `pre-commit run --all` made this mypy complain
about unused ignores in 2 files. These are false positives. Since I
don't want to maintain our all dependencies for pre-commit's mypy as
well, I made the rules for mypy a bit less strict, namely via
`--no-warn-unused-ignores`. This effectively means that mypy via
pre-commit checks less sophisticated than `poetry run mypy .` but better
not checking at all.

See:
https://github.com/python/mypy?tab=readme-ov-file#integrations
https://github.com/pre-commit/mirrors-mypy?tab=readme-ov-file ([`6dc3601`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6dc36015d76a85397640dfc136e33b9a043ecfed))

* chore: run isort across the project

Namely:

    poetry run isort --profile=black .

What also works is ruff:

    poetry run ruff check --select I --fix

To get that out of the way when autoformatting the files I will work on. ([`a4a0a31`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a4a0a31c3a159cf39926182f649b41718f095508))

### Features

* feat: Extend `Vector3d`

Give it capabilities that formerly was split between `Orientation` and
`Position` types. We decided to consolidate those into Vector3d because,
technically, all of them are just 3-vectors. The semantic
differentiation may be nice on paper but turned out to be unnecessarily
complex and all-over-the-place. Instead, have one powerful vector3d type
for all of them.

Also allow for some new, common functions like negation and substractions.

Don't add this geometricalgebra stuff and related to it, though. ([`59e813c`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/59e813c71ce191f67c266e8887dbffeebf819d62))


## v0.8.0 (2025-01-09)

### Features

* feat: add flag to control ssl and improve the logging (#20) ([`e302fa7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e302fa79f375e9ec1616c5a1e2b5ede9347bb7e3))


## v0.7.0 (2025-01-09)

### Features

* feat: add documentation to the examples (#21)

Co-authored-by: Christoph Biering <1353438+biering@users.noreply.github.com>
Co-authored-by: Marielle Muschko <marielle.muschko@wandelbots.com> ([`6b43084`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6b4308407e2eb0f03ad6321e829f8ce6d5abe7ec))


## v0.6.1 (2025-01-08)

### Fixes

* fix: provide autogenerated API from nova.api (#19)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`d84a842`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d84a842c975c5d4e890f1c65b88fbc9b8610018b))


## v0.6.0 (2025-01-07)

### Features

* feat: added .activated_motion_groups to fetch all motion groups from con (#18)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`beac2e3`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/beac2e3a45b2143c6ed9fe4a2b1fd810b5f11b0e))


## v0.5.0 (2025-01-06)

### Features

* feat(RPS-1034): added mypy check to CI * updated API * added pose transform (#17)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`8838bbf`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8838bbf4b60ab5475ef72a48700384b33a43beda))


## v0.4.0 (2024-12-23)

### Features

* feat(RPS-1027): separated plan and execute in motion group (#16)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`69da340`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/69da34055c00ab5c5917b6e32e1c2fa6e8f9dc16))


## v0.3.0 (2024-12-23)

### Features

* feat(RPS-1004): handle https, add some convenience methods (#15)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`e46cc90`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e46cc901923d290a99a06cc8fc5fa08ff6ee4502))


## v0.2.3 (2024-12-20)

### Chores

* chore: added yamllint (#14)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`4c9b66e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4c9b66eea27e25666d09d5ab70352988edacb8d4))


## v0.2.2 (2024-12-19)

### Chores

* chore: make motion_group plan call public (#13)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`8c464ba`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8c464ba5c3c05f59b68bbb0458db11f3e2422c97))


## v0.2.1 (2024-12-19)

### Chores

* chore: updated nova interface and simplified examples (#12)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`2e096bb`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2e096bb326094345bd6c30f922a5593ca46224e1))


## v0.2.0 (2024-12-17)

### Features

* feat(RPS-999): activate motion groups seperatly (#11) ([`e7a77ba`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e7a77baaa9fed33f151e48103b7bc0f5d3b04c58))


## v0.1.9 (2024-12-13)

### Chores

* chore: add pre-commit (#10) ([`44e7b6d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/44e7b6de2a9dd179ce1ea9afa1277cc077cee497))


## v0.1.8 (2024-12-12)

### Fixes

* fix: not always use "Flange" as a parameter for tcp in _load_planned_motion (#9)

Co-authored-by: Ronny Kaiser <ronny.kaiser@wandelbots.com> ([`9eb7ef1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9eb7ef1ccd3a7c21618c8cf3c3c156928606299a))


## v0.1.7 (2024-12-11)

### Chores

* chore: don't publish pi from nova package (#8)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`32f7404`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/32f740427b95abeed7ff63564a5912fb0ed17f37))


## v0.1.6 (2024-12-10)

### Chores

* chore: updated README and env variable handling (#7)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`a30cfd3`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a30cfd34e4d36e78316b37b1e8dd61ce0cb47f1e))

* chore: add examples and refactoring (#5)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`66f8a6e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/66f8a6e71d2069ccc754f56c97a928da55d18598))

* chore: add license file (#6)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`dc9401b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/dc9401bfa5bd2239feb612f71b122e2eb59cd452))


## v0.1.5 (2024-12-06)

### Chores

* chore: updated examples link ([`9d0b858`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9d0b8589bf0ee34438cf6b39b8e519080fe622e1))


## v0.1.4 (2024-12-06)

### Chores

* chore: try wandelbots-nova for pypi upload ([`a583702`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a5837020d9c2640e5646ec78cdf269d13c1fadb8))


## v0.1.3 (2024-12-06)

### Chores

* chore: try nova-python for pypi upload ([`09fa372`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/09fa3726ac256d0a9e26dcaa4f1bbaa6da956101))


## v0.1.2 (2024-12-06)

### Chores

* chore: try novapy for pypi upload ([`f93107b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f93107b49a7cb1eb48ddb47cc936f5058bbaca36))


## v0.1.1 (2024-12-06)

### Chores

* chore: updated README.md ([`1ab8485`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1ab8485d451bdf9cb55d3706a4d62cd916b25442))


## v0.1.0 (2024-12-06)

### Chores

* chore: added editorconfig * fixed poetry build ([`6f89ce8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6f89ce8ad9443a73b8198475b3ba50c4b69636f6))

* chore: CI fixes ([`9b5a7de`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9b5a7de1cea2472cd1cbff909f49d63b8b519188))

* chore: only run integration test manually for now ([`1352938`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1352938a619f2dc0e20f6a79a8c81d23e8a0356f))

### Features

* feat: implement examples & improved type interfaces & motion interface (#4)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`3273dfe`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3273dfebc5b4463618120cef00f98a1f3b87ed01))

* feat(examples): added examples to show the usage of nova-python-sdk (#2)

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`f08f793`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f08f793df329cfd43465ebcf25ff4b49bdf864ff))

### Unknown

* added integration test ([`4af7329`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4af7329fa8f3633eb7f6ab2214101c5ea021642c))

* move the robot via nova api calls (#1)

* move the robot via nova api calls

* some improvements

---------

Co-authored-by: cbiering <christoph.biering@wandelbots.com> ([`f1dee92`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f1dee925636eb129b63b32cd1eaf358265af3c4a))

* WIP ([`09e6ded`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/09e6ded6595df91d64f9b7f2da53274e11567434))

* WIP ([`940dfee`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/940dfee59ba30e42a6c733b149977eb6969aeaa3))

* WIP ([`839fc72`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/839fc7247286d4cfbdbd8f31246ab28ad475010b))

* init ([`e8a0540`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e8a0540dcc4ee41adc1f62d0d1ac2cd97ce6106a))

* Initial commit ([`22e990a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/22e990ab445a0a07d2982a56dd550269ef663f7e))
