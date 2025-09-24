# CHANGELOG


## v2.18.0 (2025-09-24)

### Features

- **NDX-228**: Trajectory cursor debug (#281)
  ([#281](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/281),
  [`9eaf965`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9eaf965e216d1dd4587af016ef6ab65f6bba5354))


## v2.17.1 (2025-09-24)

### Bug Fixes

- Default port (#286) ([#286](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/286),
  [`b510611`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b5106111416b13c7f0489857ea7e5a3a59bee772))


## v2.17.0 (2025-09-23)

### Features

- Upgrade Python SDK to Python v3.11+ (#285)
  ([#285](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/285),
  [`c6efc71`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c6efc718f2928623fcdfe3eb34d503d644f7d783))


## v2.16.3 (2025-09-23)

### Chores

- **deps**: Bump antlr4-python3-runtime from 4.13.1 to 4.13.2 (#205)
  ([#205](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/205),
  [`327dccb`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/327dccbab82d33512e457c562fab3e584b0c1692))

- **deps**: Update typer[all] requirement from <0.17,>=0.12 to >=0.12,<0.20 (#283)
  ([#283](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/283),
  [`8d9eb45`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8d9eb455a45df9cdce6a859dd26a3e90d2de8950))


## v2.16.2 (2025-09-23)

### Chores

- **deps**: Bump actions/setup-python from 5 to 6 (#276)
  ([#276](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/276),
  [`34cbd9e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/34cbd9e9069dd6720be09ad184f307574aaae4a0))


## v2.16.1 (2025-09-14)

### Bug Fixes

- Add custom nats data and add program app name to the program (#273)
  ([#273](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/273),
  [`e2e0c6d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e2e0c6d3163b10e88885796bb9d90292cd40e5ee))


## v2.16.0 (2025-09-04)

### Features

- Added from euler util function (#272)
  ([#272](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/272),
  [`1ab802b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1ab802bcc64927a907c2f3358e26459d4432c732))


## v2.15.1 (2025-09-02)

### Bug Fixes

- Only when token exists build connection string (#271)
  ([#271](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/271),
  [`048c365`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/048c36519dd897354d0fe01bd7afe21c42e0a644))


## v2.15.0 (2025-09-02)

### Features

- **NDX-231**: Visualize current debug position in code editor (#269)
  ([#269](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/269),
  [`9de584e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9de584e781817c86559233695acf61af8eba513d))


## v2.14.1 (2025-08-29)

### Bug Fixes

- Add none check for the robot cell (#268)
  ([#268](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/268),
  [`9772589`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/97725895160aa295b32c3fbeeba18ea5144017a9))


## v2.14.0 (2025-08-29)

### Features

- Publish program run to nats (#249)
  ([#249](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/249),
  [`39df504`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/39df504156e224f5f40ed5b98daa5e2cda7167d6))


## v2.13.0 (2025-08-28)

### Features

- **NDX-242**: Vscode extension to read the current robot pose and insert at cursor (#264)
  ([#264](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/264),
  [`bb05a81`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bb05a81c9b2335051aaf16723e3b4bf14c22c8db))


## v2.12.2 (2025-08-27)

### Bug Fixes

- Preserve hash characters in KUKA IO names (#266)
  ([#266](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/266),
  [`f606baf`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f606baf6c9fc9192e1a5a58b2d2994b23f221c53))


## v2.12.1 (2025-08-25)

### Bug Fixes

- **NDX-250**: Prevent index error when TrajectoryBuilder initialized without settings (#265)
  ([#265](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/265),
  [`8dbe7c6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8dbe7c649ec75a8e4803ac2d2d62ecb4cec6ac55))


## v2.12.0 (2025-08-22)

### Features

- **NDX-216**: Upgrade generated Wandelbots NOVA API client to 25.7 (#263)
  ([#263](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/263),
  [`f571aa9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f571aa98f4182aada61c77fae5407c6d09607c83))


## v2.11.0 (2025-08-19)

### Features

- **NDX-227**: Added vscode extension with basic setup to wandelbots-nova (#261)
  ([#261](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/261),
  [`f1f38a1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f1f38a192b5f69a3bd4f5cefc869795137b17aa5))


## v2.10.0 (2025-08-19)

### Features

- Update rerun-sdk version to 0.24.1 in optional dependencies (#262)
  ([#262](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/262),
  [`bce59ca`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bce59caf66d172c025deee7a246e427628420c48))


## v2.9.0 (2025-08-14)

### Features

- **NDX-232**: TrajectoryBuilder .sequence should handle *args (#260)
  ([#260](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/260),
  [`bce1239`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bce1239f03ff2b3a0cfba520f8bcf856446d9a0d))


## v2.8.7 (2025-08-13)

### Bug Fixes

- **NDX-172**: Fixed missing api autocompletion in IDE (#259)
  ([#259](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/259),
  [`d3c837e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d3c837ee936870f8640a8ca6c938ed5e27a71f1b))


## v2.8.6 (2025-08-12)

### Chores

- Upgraded dev dependencies (#258)
  ([#258](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/258),
  [`d8b09fa`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d8b09fad1628e7d60e6f6c069cc75b7287f97ddc))


## v2.8.5 (2025-08-12)

### Chores

- **deps**: Bump actions/checkout from 4 to 5 (#253)
  ([#253](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/253),
  [`9fe9029`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9fe9029d6ab9ff375bc55168c1b8d05150a6e57c))


## v2.8.4 (2025-08-12)

### Bug Fixes

- Re-enabled Wandelscript unit tests & fixed them (#257)
  ([#257](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/257),
  [`9e7bd5a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9e7bd5a7aab7a28f00c340e755426fdd88d236f5))


## v2.8.3 (2025-08-12)

### Chores

- **deps**: Update typer[all] requirement from <0.16,>=0.12 to >=0.12,<0.17 (#204)
  ([#204](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/204),
  [`100354d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/100354de31079b68b1b2fde307033b5caa8cf8ed))


## v2.8.2 (2025-07-31)

### Bug Fixes

- Use . instead of : (#248) ([#248](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/248),
  [`4c2bcae`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4c2bcae2035ac352afaff5276f801bbe4663b25b))


## v2.8.1 (2025-07-31)

### Bug Fixes

- Use . instead of : for novax programs (#247)
  ([#247](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/247),
  [`ff78f77`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/ff78f7764c72202641edbe0c25ae76811e811985))


## v2.8.0 (2025-07-30)

### Features

- Read nats_broker env for cycle related nats client (#246)
  ([#246](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/246),
  [`e2861f4`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e2861f41b8cab74d5c6f30b81be3427cb4c83eb9))


## v2.7.4 (2025-07-30)

### Bug Fixes

- Update nats client version (#245)
  ([#245](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/245),
  [`4484d98`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4484d987af03d8b6fe7895c83dc7625402524f06))


## v2.7.3 (2025-07-25)

### Bug Fixes

- Removed program source & introduced wandelscript program creation (#242)
  ([#242](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/242),
  [`5289423`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/528942351fe8d6861998e42d4abaf279e22aa2aa))


## v2.7.2 (2025-07-24)

### Chores

- **NDX-159**: Align readme with docs.wandelbots.io (#240)
  ([#240](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/240),
  [`dd6c594`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/dd6c594a9ea5bbe9a428d2228a2112b7a9e7164a))


## v2.7.1 (2025-07-24)

### Bug Fixes

- Update the program bucket novax uses (#241)
  ([#241](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/241),
  [`b92c954`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b92c9548d75652e2eb7424d3624df30de146de45))


## v2.7.0 (2025-07-24)

### Features

- **NDX-174**: Implemented TrajectoryBuilder (#236)
  ([#236](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/236),
  [`e34db72`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e34db721c9d6b9e152b862b7ce91a752cfd19ff3))


## v2.6.3 (2025-07-24)

### Bug Fixes

- Novax should not create bucket and use cell variable (#239)
  ([#239](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/239),
  [`9abb2fb`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9abb2fb17b41729f33489f5e42ad6e1985abda8f))


## v2.6.2 (2025-07-24)

### Bug Fixes

- Pin to nats 2.10 (#238) ([#238](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/238),
  [`893fc25`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/893fc25a9c6d120434daa8051e3a6e1b7eb4dcd2))


## v2.6.1 (2025-07-23)

### Bug Fixes

- Update nats connections settings for novax (#237)
  ([#237](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/237),
  [`7ce45b8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7ce45b84da8e65391ef1d3814cf161d9a0ab4d1b))


## v2.6.0 (2025-07-22)

### Features

- Add program store (#235) ([#235](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/235),
  [`47f3196`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/47f31966759674957687b331020f446a7ee44840))


## v2.5.0 (2025-07-17)

### Features

- Added deregister_program to novax (#234)
  ([#234](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/234),
  [`0e75f9f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0e75f9f30e005907fb8dc91ce890780623d6bdef))


## v2.4.1 (2025-07-16)

### Bug Fixes

- Update example and rerun address for VS Code integration (#233)
  ([#233](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/233),
  [`cf4957b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/cf4957bfc8bbb492617e3cec477a7308c7594025))


## v2.4.0 (2025-07-15)

### Features

- Improved program metadata & API description (#232)
  ([#232](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/232),
  [`cb435a0`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/cb435a0f95995a708a27e08e39a716d8934b6790))


## v2.3.1 (2025-07-14)

### Bug Fixes

- Cleanup novax (#229) ([#229](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/229),
  [`7607c83`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7607c831970b77e6b49ed12fd8de73d56a2ae195))


## v2.3.0 (2025-07-14)

### Features

- Removed /runs resource from Nova API (#226)
  ([#226](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/226),
  [`6f4d35f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6f4d35fd670d611c3421e950dd210190a01c32f3))


## v2.2.2 (2025-07-11)

### Bug Fixes

- Novax-docs-at-root (#228) ([#228](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/228),
  [`fdfb511`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/fdfb51153ef6a6a4cd8fff9e675eb6dbf47d643f))


## v2.2.1 (2025-07-11)

### Bug Fixes

- Add root path (#227) ([#227](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/227),
  [`c9e5986`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c9e5986ae44f149c95abe6cdf34b19e0eb889d30))


## v2.2.0 (2025-07-09)

### Features

- Added register_program_source to novax (#224)
  ([#224](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/224),
  [`2bf0da9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2bf0da96299f04bef0ca5a0a7054313955597242))


## v2.1.2 (2025-07-08)

### Bug Fixes

- Nova-rerun-client-access (#222)
  ([#222](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/222),
  [`a05ca61`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a05ca61bce1d74c36bbc2f67f5716956b2194ce7))


## v2.1.1 (2025-07-08)

### Bug Fixes

- Remove wandelscript await (#221)
  ([#221](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/221),
  [`45acf03`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/45acf036d8d3ca3fa1fcf3d4c1cd5bc9a2ec5b29))


## v2.1.0 (2025-07-07)

### Features

- **NDX-104**: Introduced Novax app framework (#217)
  ([#217](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/217),
  [`b713415`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b713415debc237ab201b5927cac43787f923c1ec))


## v2.0.0 (2025-07-07)

### Features

- Add viewer to decorator for transparent rerun logs (#216)
  ([#216](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/216),
  [`4d548ca`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4d548ca8de7784c2f4676073f97e176742b6c85d))


## v1.25.0 (2025-07-04)

### Features

- Save rerun file instead of streaming on vs code viewer (#210)
  ([#210](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/210),
  [`f091e57`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f091e576ed11598b0388fe1961602605c9f57ece))


## v1.24.2 (2025-07-03)

### Chores

- Updated dependabot action to 10.2.0
  ([`62229c9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/62229c97236cb5d13a7ace647189570d317b4d25))


## v1.24.1 (2025-07-02)

### Chores

- Update nova to 25.6 (#218) ([#218](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/218),
  [`8080775`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/808077506b74fb3eb8d04e925cb79af70cf5cedc))


## v1.24.0 (2025-06-27)

### Features

- Removed enumeration from example files & improved list in readme (#209)
  ([#209](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/209),
  [`5edd84d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5edd84d9ee0c76af422a4a058aa40539d8bf5797))


## v1.23.0 (2025-06-26)

### Features

- Removed ensure_virtual_robot_controller & updated examples (#208)
  ([#208](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/208),
  [`8707dc8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8707dc86214f8a9b49e7d0e1e3ae8f02bf4255df))


## v1.22.0 (2025-06-26)

### Features

- Added removed ws examples to integration tests (#207)
  ([#207](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/207),
  [`06dd8e2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/06dd8e2f37469bd69b5ad1176e8edd5acfcaa568))


## v1.21.0 (2025-06-26)

### Features

- Add json and position parameters to add_virtual_robot_controller (#178)
  ([#178](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/178),
  [`5249029`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5249029057e88986b5f3f1fcbdb8f42d324108df))


## v1.20.0 (2025-06-25)

### Features

- **NDX-67**: Add declarative controller creation to @nova.program decorator (#195)
  ([#195](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/195),
  [`bc50be2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bc50be246cabd01b0f00e1eaa936026b83dd2d2d))


## v1.19.0 (2025-06-24)

### Features

- **CI**: Increased cell creation timeout for integration test (#199)
  ([#199](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/199),
  [`c893418`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c893418a8d2e83c8c3b808472d5e3d7da8a2d3e3))


## v1.18.4 (2025-06-24)

### Bug Fixes

- More unique sandbox name when creating an instance (#206)
  ([#206](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/206),
  [`e3c1399`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e3c139978b2a154df29b5efd48f171384cd6299b))


## v1.18.3 (2025-06-23)

### Bug Fixes

- Migrate to rerun 0.23 (#202) ([#202](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/202),
  [`096d326`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/096d3266f3820e0ddefd6ad54f7d625a105eaa00))


## v1.18.2 (2025-06-20)

### Chores

- Update rerun dependency (#200)
  ([#200](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/200),
  [`59bb1c6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/59bb1c666e36ca2fbed907ae988879398de76fe4))


## v1.18.1 (2025-06-18)

### Bug Fixes

- **NDX-90**: Cleanup and make wandelscript pkg optional (#197)
  ([#197](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/197),
  [`072d83c`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/072d83c0ac499ef31e90fd0049ab563aee26a212))


## v1.18.0 (2025-06-18)

### Features

- **NDX-90**: Move Wandelscript to SDK repository (#190)
  ([#190](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/190),
  [`ff1a9a1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/ff1a9a16ed53f4e19d75695f592d15f59f5b3e66))


## v1.17.1 (2025-06-17)

### Bug Fixes

- Install `pdoc` into the venv on `autodocs.yml` (#196)
  ([#196](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/196),
  [`4390fca`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4390fca69b8c85f958cb9e019c8fa3f7762c237d))


## v1.17.0 (2025-06-17)

### Features

- Add ensure_virtual_tcp function (#189)
  ([#189](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/189),
  [`0624326`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/062432699effc77b44f826ea90659a402a325c3c))


## v1.16.0 (2025-06-12)

### Features

- **NDX-80**: Improve nova module import performance with lazy loading (#187)
  ([#187](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/187),
  [`5e878a4`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5e878a4ee8b2a3fe08c6170f77eeb74fcd975c3b))


## v1.15.3 (2025-06-11)

### Bug Fixes

- **NDX-76**: #5 fixed job that creates a release wheel
  ([`bf5f3a5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bf5f3a5c7cbaa2c4d0511afbae3ae108151c366e))


## v1.15.2 (2025-06-11)

### Bug Fixes

- **NDX-76**: #4 fixes to the release CI
  ([`c91116f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c91116f42de1a64032f84ccd3a0c2fa1b0534606))


## v1.15.1 (2025-06-11)

### Bug Fixes

- **NDX-76**: #3 fixed CI to release on release/ branches
  ([`1cee9e1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1cee9e1228f16f12b5310c45aeb79fdad03ff1bc))


## v1.15.0 (2025-06-11)

### Features

- Standardize controller creation pattern across examples (#186)
  ([#186](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/186),
  [`14be7bc`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/14be7bc893583206f696cc7887b8c0ae7d65dcf7))


## v1.14.2 (2025-06-11)

### Bug Fixes

- **NDX-76**: #2 create build number + branch slug
  ([`98307df`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/98307dfdaf26e08681d3c1a3753bd66baf76d30a))


## v1.14.1 (2025-06-11)

### Bug Fixes

- **NDX-76**: Added semantic version handling
  ([`bbb9744`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bbb97442e78b046ead079a2b7b6baad2b625ca93))


## v1.14.0 (2025-06-10)

### Features

- **NDX-76**: Add action to release from release/** branch (#185)
  ([#185](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/185),
  [`e0ac563`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e0ac56390c4a71e32d87d2685947d9ac826a3e10))


## v1.13.3 (2025-06-06)

### Chores

- Add AGENTS guidelines (#184) ([#184](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/184),
  [`4fda575`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4fda575e6baef9d55724402696f24565c5d1483a))


## v1.13.2 (2025-06-06)

### Chores

- Updated readme to create dev builds (#183)
  ([#183](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/183),
  [`8f35545`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8f355459f04960587605616e0bd50a0d72fc4ea6))


## v1.13.1 (2025-06-06)

### Chores

- Improved nightly build
  ([`c6a3b73`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c6a3b73c2537ae9014a3d5c1c168836bd7fb3074))


## v1.13.0 (2025-06-06)

### Features

- Build wheel in feature branches (#182)
  ([#182](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/182),
  [`37f4a4b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/37f4a4b50e9fd775c029f07f6b2c1b8db53be797))


## v1.12.0 (2025-06-06)

### Features

- **NDX-76**: Introduce nightly release (#180)
  ([#180](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/180),
  [`4630ee0`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4630ee0a3457bf3ba87edc0141ad214bafe63ad2))


## v1.11.0 (2025-06-04)

### Features

- **NDX-17**: Introducing cycle events, measurement and NATS propagation thereof (#169)
  ([#169](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/169),
  [`550d799`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/550d79901be7695dc98022e4ed9b4938de420f69))


## v1.10.4 (2025-06-04)

### Bug Fixes

- **NDX-75**: Added scheduled job that cleans up all remaining instances (#179)
  ([#179](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/179),
  [`3989ca7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3989ca7a3e5c7a45729157c12e453baa127f80e7))


## v1.10.3 (2025-06-04)

### Bug Fixes

- Skip valid token check if a username and password is set (#176)
  ([#176](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/176),
  [`e3aee8a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e3aee8a7e2a90ecd2c0ec5a9a1dc9245506f7f95))


## v1.10.2 (2025-06-03)

### Bug Fixes

- Fixed uv_runner validation (#177)
  ([#177](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/177),
  [`5c31d82`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5c31d8287bb6354a6f6ebd30202d43124b8eba69))


## v1.10.1 (2025-06-03)

### Chores

- Updated readme (#175) ([#175](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/175),
  [`5c3edd6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5c3edd61e28dddf8c66607c570a2040bea495cfc))


## v1.10.0 (2025-05-28)

### Features

- Add-test-case-for-action-serialization (#174)
  ([#174](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/174),
  [`9e36f8d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9e36f8d9227a7b02d0b188ca114bfaa202b548a9))


## v1.9.1 (2025-05-28)

### Chores

- **nova**: Upgrade nova dependency to 25.4.0 (#165)
  ([#165](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/165),
  [`954482a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/954482a5db28b6b01644a854ddb910d7cdc19dcd))


## v1.9.0 (2025-05-27)

### Features

- **runner**: Added result field (#173)
  ([#173](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/173),
  [`7bca3e8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7bca3e82e22d9b2d127a17a515201959249b7246))


## v1.8.2 (2025-05-23)

### Chores

- Improve runner motion state recording & cleanup (#168)
  ([#168](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/168),
  [`d1067f8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d1067f8f3d6563917c6bdc925ec8c00171c7dd8c))


## v1.8.1 (2025-05-23)

### Chores

- Cleanup state enum (#167) ([#167](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/167),
  [`b5b38c8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b5b38c823b5b3e3d1a4ae069b42216b05a849642))


## v1.8.0 (2025-05-21)

### Features

- Track runner motion state result (#166)
  ([#166](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/166),
  [`bccdb8d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bccdb8d0ae775c597a1ec3d3358aece2d8d67851))


## v1.7.0 (2025-05-21)

### Features

- **NDX-37**: Update environment variables to use prod (#163)
  ([#163](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/163),
  [`a22ec11`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a22ec119c04fa350709f367aea8b7db57902a779))


## v1.6.2 (2025-05-20)

### Bug Fixes

- Remove loguru logger (#164) ([#164](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/164),
  [`5fd1cec`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5fd1cecba37044b3167dcbf7988d94c7c0cabc95))


## v1.6.1 (2025-05-20)

### Bug Fixes

- **NDX-45**: Improved integration test CI & open robot_cell before running (#162)
  ([#162](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/162),
  [`3c89d61`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3c89d61d4bd3422f2e4d0ecebcce273a8460388a))


## v1.6.0 (2025-05-15)

### Features

- Expose combine_trajectories in the module exports (#161)
  ([#161](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/161),
  [`34dde3d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/34dde3dc95e68394ecb7db5c58b9828119bef106))


## v1.5.4 (2025-05-14)

### Bug Fixes

- Pass the collision scene to non collision motions (#160)
  ([#160](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/160),
  [`e2d2f1b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e2d2f1bbde945346da6669ce7deaeaf2a5396459))


## v1.5.3 (2025-05-14)

### Chores

- Make test pass (#158) ([#158](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/158),
  [`02ea30e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/02ea30edb07179a82d25a60a8518160c3da9dc93))


## v1.5.2 (2025-05-13)

### Bug Fixes

- Show collision of robots while using multiple robots (#159)
  ([#159](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/159),
  [`6e9764b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6e9764b131b763289e42572ecfc9e11feab4d62c))


## v1.5.1 (2025-05-13)

### Bug Fixes

- **RPS-1380**: Upgrade pydantic
  ([`efbf8c2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/efbf8c2d3843ff85dcc090cd11c9eddb0cda74f4))


## v1.5.0 (2025-05-12)

### Features

- Enhance log_actions method to support optional motion_group par… (#156)
  ([#156](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/156),
  [`928ffc8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/928ffc8adb79232a7f5c58a58184bbc837bba0df))


## v1.4.0 (2025-05-09)

### Features

- **RPS-1615**: Implemented program runner for Python program (#153)
  ([#153](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/153),
  [`0838520`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/083852080ac535bf824f33bcfa8754676e52f2aa))


## v1.3.1 (2025-05-08)

### Bug Fixes

- Include WaitAction in ActionContainerItem and skip in trajectory… (#155)
  ([#155](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/155),
  [`b89f6e9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b89f6e9b9223071a0711a5836517c02e81ae9fe9))


## v1.3.0 (2025-05-08)

### Features

- Add FeedbackCollision error visu (#154)
  ([#154](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/154),
  [`2cfe834`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2cfe8342d2673152f7b49904f3a68130a5452676))


## v1.2.0 (2025-05-08)

### Features

- Add wait action (#152) ([#152](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/152),
  [`8d3968a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8d3968a4e5abfc20626663948a9dbc9ff8fa432b))


## v1.1.0 (2025-05-06)

### Features

- Add a example on how to serialize a program (#145)
  ([#145](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/145),
  [`7185c54`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7185c54116f9deca85deaee53dc688528a4059d7))


## v1.0.0 (2025-05-05)

### Bug Fixes

- **RPS-1595**: Tcp_pose is returned with the correct TCP reference now (#148)
  ([#148](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/148),
  [`cbabf5d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/cbabf5df864f3187b2e2be8414cc7fa0ab652d78))


## v0.53.1 (2025-04-30)

### Bug Fixes

- Correct typo in motion group jogging API variable name (#147)
  ([#147](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/147),
  [`eea91f2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/eea91f2b141969adb3e0fc687eb2fd3823be522a))


## v0.53.0 (2025-04-30)

### Features

- Add motion group jogging API to ApiGateway (#146)
  ([#146](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/146),
  [`fa2ea4b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/fa2ea4b0d027d6ff1f8e8e0965a3c8b954ba15eb))


## v0.52.0 (2025-04-25)

### Features

- Add more supported types to render safety geometry in rerun (#144)
  ([#144](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/144),
  [`5087c98`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5087c98feec52b5cb576c9370cb874d4baae2af2))


## v0.51.0 (2025-04-25)

### Features

- Set mounting pose of safety zone (#143)
  ([#143](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/143),
  [`0e52cd8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0e52cd83a88d24759f86fb2909655e77e6b18f80))


## v0.50.3 (2025-04-24)

### Bug Fixes

- Update workflow name for rerun image to match standard format (#142)
  ([#142](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/142),
  [`6a53cc0`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6a53cc0d9a6f6bc63640fbf9cec23e043d004128))


## v0.50.2 (2025-04-24)

### Bug Fixes

- Special chars are not allowed (#141)
  ([#141](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/141),
  [`b1ed2da`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b1ed2da4091b4c734bca29458bd72fbe3a54a033))


## v0.50.1 (2025-04-24)

### Bug Fixes

- Specify correct workflow name for rerun bridge builds (#140)
  ([#140](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/140),
  [`0e919c6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0e919c601ca975c7edf428493e249c43129606a9))


## v0.50.0 (2025-04-17)

### Features

- **rps-1560**: Add support for pysical controllers (#139)
  ([#139](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/139),
  [`8024d64`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8024d64cc8fb9b5761a28df3298a41ac247967b1))


## v0.49.0 (2025-04-15)

### Features

- **RPS-1509**: Introduce a way to run a standalone Python robot program via uv (#138)
  ([#138](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/138),
  [`54ca815`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/54ca815e6767f549439389575b4a7aa8c3f3c0cc))


## v0.48.1 (2025-04-14)

### Chores

- **RPS-1557**: Adjust CI files for `uv`
  ([`86fe6c3`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/86fe6c33049d3de2df7c2e970d3fce5d2ce15916))

- **RPS-1557**: Adjust files in `nova_rerun_bridge` to `uv`
  ([`0de3c5b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0de3c5bc5a07522b212565eb6326dd526d34c277))

- **RPS-1557**: Adjust files to `uv`
  ([`a429599`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a429599443289f0df71c84d3bedd46f9b90acfd3))

- **RPS-1557**: Migrate from `Poetry` to `uv`
  ([`aea47b4`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/aea47b47dd91d19ced2f6b2fab9d1fb7a88156ce))


## v0.48.0 (2025-04-10)

### Features

- **rps-1476**: Isolate api client usage to api gateway class (#136)
  ([#136](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/136),
  [`c398467`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c39846728fd983b84c5d23e9e94ea32c8acc35ab))


## v0.47.12 (2025-04-05)

### Chores

- Upgrade major version to 1.x.x (#134)
  ([#134](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/134),
  [`ac4ef09`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/ac4ef09a0cb16fadb4339c4e17d3973df6bf914d))


## v0.47.11 (2025-04-03)

### Bug Fixes

- **pose**: Pose supports model_validate (#133)
  ([#133](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/133),
  [`7db0314`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7db0314b501b0800195551c252976edb6c862a2d))


## v0.47.10 (2025-03-25)

### Bug Fixes

- **RPS-1217**: State streaming (#128)
  ([#128](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/128),
  [`81ee4e0`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/81ee4e02674f2827bbfd5a98a516c2fcac162f6c))


## v0.47.9 (2025-03-20)

### Bug Fixes

- **pdoc**: Added google style to pdoc generation (#126)
  ([#126](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/126),
  [`780e615`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/780e615952f2ca1d9262c40d39abbfede2e64d5d))


## v0.47.8 (2025-03-20)

### Bug Fixes

- **RPS-155**: Provide motion group / robot ID on `PlanTrajectoryFailed` exceptions
  ([`e767c9c`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e767c9c3f6765a076f026ebf706d8dac75312f5c))


## v0.47.7 (2025-03-20)

### Chores

- **deps**: Bump actions/setup-python from 4 to 5 (#101)
  ([#101](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/101),
  [`7a96381`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7a96381d3097297ba3c0bf5d780a92c86756b657))


## v0.47.6 (2025-03-20)

### Chores

- **RPS-1042**: Updated README (#124)
  ([#124](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/124),
  [`dfd41da`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/dfd41da641731211be2a2d9d93f7b3baa9422830))


## v0.47.5 (2025-03-19)

### Bug Fixes

- **RPS-1312**: Limit TPC default velocity to 50mm/s
  ([`099a923`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/099a923d2343b399b12823957f7b31375f65ef60))

### Chores

- Sort imports
  ([`22b7132`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/22b7132d656e10b516c4726cab6210c7046b07f7))


## v0.47.4 (2025-03-18)

### Chores

- Remove some superluous else/elif statements
  ([`89a18ab`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/89a18ab5e3b1331e8acb6a5c1b9fd2dd49cbe108))


## v0.47.3 (2025-03-17)

### Chores

- Add VSCode colors for the Nova project
  ([`f5627c7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f5627c78310a1c14ff6134d3972b1d90461b57cb))

- Simplify a docstring in `motions.py`
  ([`b1299fa`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b1299faa5e43951180da31ddaf2383e4a38241e9))

- **RPS-1310**: Use verbose names for actions
  ([`2897f2e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2897f2e9c7bba71366fbda817ce0d3f4997fe77f))


## v0.47.2 (2025-03-17)

### Bug Fixes

- Update function signature and dont expose CollisionFree (#119)
  ([#119](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/119),
  [`2fbc81f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2fbc81f337a0dd16d8604090d4dc1c9bcd4c55ad))


## v0.47.1 (2025-03-17)

### Chores

- **RPS-1129**: Motion group should accessible via id str (#118)
  ([#118](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/118),
  [`6f1f8d4`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6f1f8d4abbf180f677ec687b4873248ae298c5eb))


## v0.47.0 (2025-03-12)

### Bug Fixes

- Allow for special characters in PR titles
  ([`aa4cbf8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/aa4cbf85d49f5cccdcc088bd4a30f967a3032c2b))

### Features

- **RPS-1311**: Add pretty string repr for `PlanTrajectoryFailed` errors
  ([`7758378`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/77583788800b3f0cc672d3a95f4dab6e42ce594b))


## v0.46.0 (2025-03-06)

### Features

- **RPS-1094**: Add opc ua functions (#113)
  ([#113](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/113),
  [`26b83de`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/26b83de06c1dad248baff3cbf57e4d820e26869b))


## v0.45.0 (2025-03-04)

### Features

- Embed defaults for auth (#112)
  ([#112](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/112),
  [`a79a436`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a79a43677acdba26572ec76fe7e2c63f81810ec2))


## v0.44.0 (2025-03-04)

### Features

- **rps-1164**: Add documentation to public functions (#111)
  ([#111](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/111),
  [`f610a5f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f610a5f0097099c9163e8662a9d11bdb33f46789))


## v0.43.1 (2025-03-04)

### Chores

- **deps**: Bump python-semantic-release/publish-action from 9.20.0 to 9.21.0 (#103)
  ([#103](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/103),
  [`5e9ee78`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5e9ee78af6bee4b3d237c454723bbb9713018aac))


## v0.43.0 (2025-03-04)

### Features

- Implement Auth0 default configuration and removed dotenv (#108)
  ([#108](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/108),
  [`5f0a9cc`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5f0a9cc06e8c9faf8eab93da1abf9858121c5b06))


## v0.42.2 (2025-03-03)

### Chores

- **cleanup**: Removed unused motion recording & mg_id to motion state (#110)
  ([#110](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/110),
  [`7c7b066`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7c7b0666b7fd4bda2c4abc69c0253dfba3e71f1a))


## v0.42.1 (2025-02-28)

### Chores

- Improve readme (#109) ([#109](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/109),
  [`fbea4f1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/fbea4f1760c48d87de92428242b12ef00dd798cd))


## v0.42.0 (2025-02-27)

### Features

- Add simplified robocore example (#106)
  ([#106](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/106),
  [`8fef8bb`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8fef8bbffe6187d42393634580ddbc4dbb56e65a))


## v0.41.0 (2025-02-26)

### Bug Fixes

- Add manufacturer-specific home positions for virtual robot contr… (#105)
  ([#105](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/105),
  [`56d1cf2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/56d1cf2d4588e19233b3d440c8a4b0e678298f2b))

- Remove gltf-transform dependency and update model download (#100)
  ([#100](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/100),
  [`f1a2bda`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f1a2bda017ea97b3696cdccef537c1f278d7e1c3))

- Update docker builds to poetry 2 (#96)
  ([#96](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/96),
  [`5e6f8cd`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5e6f8cdcb6a9c8c4f2e815f67eb6e9b904f4406a))

### Chores

- Relax numpy dependency to allow >1.1.19
  ([`3593327`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3593327f78c982f705a621d567d6c0afaca8c4a2))

- Simplified splitting (#91) ([#91](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/91),
  [`133e50f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/133e50f58f9eec64976ba30e07e8814750cf6875))

### Features

- Collision free benchmark (#86) ([#86](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/86),
  [`896d208`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/896d208b22abfb3beb7cb60ccb627fcd0d5c8cb7))

- Implement auto refresh of access token (#104)
  ([#104](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/104),
  [`6a50ef7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6a50ef7dac20f94ce5bc90f1a4c3f0fa638493a8))

- Stream robot state to rerun on standard timeline (#97)
  ([#97](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/97),
  [`6ff7625`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6ff7625fe3d3eedcb871abac5ee5cec4b0d41acb))

- **docs**: Add autodocs to pipeline (#92)
  ([#92](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/92),
  [`9a7f6ee`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9a7f6ee4985e0cb0cfc845c399acaf20b49191d0))


## v0.40.1 (2025-02-20)

### Bug Fixes

- Collision-free-movement-type (#76)
  ([#76](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/76),
  [`f861ff8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f861ff80b0b7b87bacff963563915ccdec131671))


## v0.40.0 (2025-02-20)

### Features

- **RPS-1253**: Use default loguru sink (#78)
  ([#78](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/78),
  [`c523ba7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c523ba765ed5f7ea956c0cd3b95c4bef3a0e7261))


## v0.39.1 (2025-02-20)

### Bug Fixes

- Fixed robotcell creation
  ([`7f7e182`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7f7e1825bb3e9b4a074d80052b9d26a8489c25fe))


## v0.39.0 (2025-02-20)

### Features

- Set virtual robot mounting setup
  ([`be0d882`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/be0d8825308fdac8e4e113ec72246572eb0516e6))


## v0.38.3 (2025-02-20)

### Bug Fixes

- Ensure optimizer setup is updated with new TCP value
  ([`2e2c36f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2e2c36f4e53a992b6a553dfa4a87dce4a9e717b2))


## v0.38.2 (2025-02-20)

### Chores

- Return robot cell from cell class * cleanup (#90)
  ([#90](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/90),
  [`ec840be`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/ec840be673453442ffe6d7542809e0bfba1cf051))


## v0.38.1 (2025-02-20)

### Chores

- Streamlined controller init (#89)
  ([#89](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/89),
  [`bff2e7d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bff2e7d95bb0f99e5c612df4c5464e81fcd4afa1))


## v0.38.0 (2025-02-19)

### Features

- **RPS-1214**: Streaming execution (#68)
  ([#68](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/68),
  [`c80fa66`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c80fa66a5dd6215852845d17a3ba88193d41f409))


## v0.37.1 (2025-02-19)

### Chores

- Added CODEOWNERS file (#88) ([#88](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/88),
  [`59043a4`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/59043a4ca86de7959344cf267e1e4ad72c77ebcd))


## v0.37.0 (2025-02-18)

### Features

- **RPS-1265**: Download and attach diagnose package on CI fail (#85)
  ([#85](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/85),
  [`30b3638`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/30b363860b7b6d16246f2bc886f776b42a8c3b37))


## v0.36.2 (2025-02-18)

### Chores

- **deps**: Bump python-semantic-release/publish-action from 9.19.0 to 9.20.0 (#80)
  ([#80](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/80),
  [`3fef1d8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3fef1d89a5956fe4c732e892a1693135c77098a0))

- **deps**: Bump python-semantic-release/python-semantic-release from 9.19.0 to 9.20.0 (#79)
  ([#79](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/79),
  [`91180a7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/91180a753b165f68e06f186f363ccba96db24f75))

- **deps**: Bump trimesh from 4.6.1 to 4.6.2 (#84)
  ([#84](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/84),
  [`6980d79`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6980d79b3a0f745dafef6544e19070dbe168e2f4))

- **deps-dev**: Bump pytest-asyncio from 0.24.0 to 0.25.3 (#54)
  ([#54](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/54),
  [`34538f5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/34538f5c19fd6a823fae06cade06cd5811595f6d))


## v0.36.1 (2025-02-18)

### Chores

- **deps**: Bump numpy from 2.2.2 to 2.2.3 (#83)
  ([#83](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/83),
  [`4e7a435`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4e7a4355caacbbded3d46499b23748a7e3105204))

- **deps**: Bump scipy from 1.15.1 to 1.15.2 (#82)
  ([#82](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/82),
  [`7fdf855`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/7fdf855eac6d01ec2e38178c497ef1ec498a2e9f))


## v0.36.0 (2025-02-17)

### Features

- **RPS-1224**: Added user-agent to api client (#66)
  ([#66](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/66),
  [`13696aa`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/13696aae25680022cc4cf7a21f0d4205682fde55))


## v0.35.0 (2025-02-14)

### Features

- Added __getitem__ in vector3d (#77)
  ([#77](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/77),
  [`4612008`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/46120080a07da5b32c5f72374c854d3275941e55))


## v0.34.0 (2025-02-14)

### Features

- Added __iter__ to vector & pose (#75)
  ([#75](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/75),
  [`0d48799`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0d487996e3b9b89c78c31e91225d763166817815))


## v0.33.1 (2025-02-14)

### Bug Fixes

- Update catalog entry name format in image push workflow (#72)
  ([#72](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/72),
  [`f4c512b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f4c512bfc2e0622792b032a4578a5d6aa0e1619c))


## v0.33.0 (2025-02-14)

### Features

- Add catalog entry update step to image push workflows (#69)
  ([#69](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/69),
  [`790e68d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/790e68d0fed8bbf1074ae1dae9dd294d9c8b03d8))


## v0.32.1 (2025-02-13)

### Bug Fixes

- Install extension in rerun app
  ([`6ba0f4d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6ba0f4d10a362be85c8ba5e45cf2151f5d04eeba))


## v0.32.0 (2025-02-13)

### Features

- Add safety zone logging functionality (#71)
  ([#71](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/71),
  [`a41103d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a41103d0241555effbe087d88cd067ad4eb119da))


## v0.31.0 (2025-02-13)

### Features

- Invert pose (#70) ([#70](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/70),
  [`84be73c`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/84be73c0793be62568b25e4f2b900e0bfefb348e))


## v0.30.1 (2025-02-13)

### Bug Fixes

- Collision free p2p example
  ([`78db5c9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/78db5c9b964e07c0c8bf673cb2874f8d0618cf80))


## v0.30.0 (2025-02-12)

### Features

- Add motion group kinematic API and example for reachability check (#67)
  ([#67](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/67),
  [`1651d7d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1651d7da70857b4f9d2535472d75544c247303bf))


## v0.29.0 (2025-02-12)

### Features

- Add collision-aware motion types and collision free actions (#61)
  ([#61](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/61),
  [`6a93ad5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6a93ad51bec2f58a9d930c109994241713f548d7))


## v0.28.0 (2025-02-12)

### Features

- **RPS-1221**: Refine nova actions interface (#64)
  ([#64](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/64),
  [`c6d112d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c6d112d2854657398d4c82fae9681a98a7d7ffe0))


## v0.27.2 (2025-02-11)

### Chores

- Update README.md (#65) ([#65](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/65),
  [`bb12fb1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/bb12fb15db8eda56b770e07bc08cd153cb80c068))


## v0.27.1 (2025-02-11)

### Bug Fixes

- Badge (#63) ([#63](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/63),
  [`2781ac3`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2781ac31ce05824bec140b1d61a13f4a097346d6))


## v0.27.0 (2025-02-11)

### Features

- Added np array support for vector3d and pose (#62)
  ([#62](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/62),
  [`5021081`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5021081fcc2df801e0c768d4e9eb3fd46e143552))


## v0.26.2 (2025-02-10)

### Chores

- **deps**: Bump actions/setup-python from 4 to 5 (#52)
  ([#52](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/52),
  [`5ae9aff`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/5ae9aff8a5ecafa0659a9eea7bcb56c01c75fc37))

- **deps**: Bump python-semantic-release/publish-action from 9.8.9 to 9.19.0 (#53)
  ([#53](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/53),
  [`60092de`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/60092de11fbae84a32674d7366ded7d4cbda05cd))

- **deps**: Bump python-semantic-release/python-semantic-release from 9.11.1 to 9.19.0 (#51)
  ([#51](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/51),
  [`3bd9c67`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3bd9c67c4e9827198e0269f95852a0d32ed7271c))


## v0.26.1 (2025-02-10)

### Bug Fixes

- Move back to home in example (#60)
  ([#60](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/60),
  [`a62def7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a62def7e6baac37f99ed6aba460d3864e4967a43))


## v0.26.0 (2025-02-10)

### Features

- Add various collider types (#58)
  ([#58](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/58),
  [`9d60c66`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9d60c665c693834abc27ecf558f5fa2c8e907f9b))


## v0.25.3 (2025-02-10)

### Bug Fixes

- Fixed nova-rerun-bridge extras (#59)
  ([#59](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/59),
  [`287e222`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/287e222fb63ed6015a6946375edde17aa44e13a1))


## v0.25.2 (2025-02-10)

### Chores

- **deps-dev**: Bump ruff from 0.8.6 to 0.9.6 (#56)
  ([#56](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/56),
  [`a813520`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a813520bff110afa6e4c0d4c0b16b8b1f51fd0d8))


## v0.25.1 (2025-02-10)

### Chores

- Lock
  ([`eb0ba0d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/eb0ba0db71ca3569d1d6834efda99886111d21a3))


## v0.25.0 (2025-02-10)

### Features

- Only plan and not execute in welding example (#57)
  ([#57](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/57),
  [`3155e6f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/3155e6fb7fd3623bee7ea9848465265761780897))


## v0.24.1 (2025-02-10)

### Bug Fixes

- Simple check for http prefix (#42)
  ([#42](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/42),
  [`cd4b478`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/cd4b478bb551f7a444ef10f696980d4a2a059067))


## v0.24.0 (2025-02-10)

### Features

- Add optional rerun bridge (#45) ([#45](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/45),
  [`00177e9`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/00177e95a056bca997f5e2cee6f59d6b1e0b3f00))


## v0.23.0 (2025-02-10)

### Features

- Enhance trajectory planning with optional starting joint position (#49)
  ([#49](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/49),
  [`2eeb747`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2eeb747bfcac6da28a1ea629396eb89c1c246cc3))


## v0.22.0 (2025-02-10)

### Features

- Added __eq__ to vector and pose (#48)
  ([#48](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/48),
  [`8198eb0`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8198eb08edf0eda150bed202fc619ec42c7c3e99))


## v0.21.0 (2025-02-07)

### Features

- Streaming execution first draft (#43)
  ([#43](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/43),
  [`11bb47a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/11bb47a94e4370c69291af59eb8b425e693a7c5d))


## v0.20.0 (2025-02-07)

### Features

- Propagate robot state in motion group (#47)
  ([#47](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/47),
  [`73e87d2`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/73e87d24cc6ebede177a074cdbf5ef76a956b9dd))


## v0.19.1 (2025-02-07)

### Chores

- Added active tcp to abstract robot (#46)
  ([#46](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/46),
  [`f64cb39`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f64cb393164cb92ae8ec5173ca32ce912c390420))


## v0.19.0 (2025-02-07)

### Features

- Get active_tcp_name & added motion_settings to var name (#44)
  ([#44](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/44),
  [`352b17b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/352b17b5ee63e3e747b7ef19f85226b644db3b07))


## v0.18.0 (2025-02-06)

### Features

- **RPS-1206**: Implemented I/O access, read & write (#41)
  ([#41](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/41),
  [`25e6caa`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/25e6caace690d607a1ea44eb72ab521e8bc35da8))


## v0.17.5 (2025-02-06)

### Chores

- Make robot cell devices readable (#40)
  ([#40](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/40),
  [`b7bbbbf`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/b7bbbbf9b2db6a0d76c99acd232a7bdeb61180d3))


## v0.17.4 (2025-02-05)

### Chores

- Enable example tests in CI (#39)
  ([#39](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/39),
  [`929db4e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/929db4e2b8d71137b4bf2f5dd20c9248ca661e30))


## v0.17.3 (2025-02-05)

### Chores

- **RPS-898**: Implemented robotcell + execute * abstract robot and deps (#38)
  ([#38](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/38),
  [`45eec6a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/45eec6aad6fb5576210176e2eab1d0d062e33fff))


## v0.17.2 (2025-02-05)

### Chores

- Update wandelbots_api_client to version 25.1.0 (#37)
  ([#37](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/37),
  [`ec3fd3d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/ec3fd3d4482c900915f0a32f3cb763529d585618))


## v0.17.1 (2025-02-04)

### Bug Fixes

- Exception naming (#36) ([#36](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/36),
  [`723b2d8`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/723b2d8fff88a32c44003b6e3bbf4499396f7890))


## v0.17.0 (2025-02-04)

### Features

- Make authorization flow asynchron. (#34)
  ([#34](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/34),
  [`2e0dae6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2e0dae6b1c2ddb56ff66cbcf4a3033165a414a55))


## v0.16.1 (2025-02-04)

### Bug Fixes

- **RPS-1174**: Dont deactivate motion groups (#35)
  ([#35](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/35),
  [`67b6535`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/67b6535a3f519b74807ab54db0df4e48458a06a5))


## v0.16.0 (2025-01-23)

### Features

- Expose error object in exception (#32)
  ([#32](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/32),
  [`0b7351f`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0b7351f3520bf96e5331c1693c7e6dd2c8d477da))


## v0.15.1 (2025-01-22)

### Chores

- Improved example descriptions (#33)
  ([#33](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/33),
  [`c73faa5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/c73faa5669de8e1d98c6057ebee40783749cb83c))


## v0.15.0 (2025-01-22)

### Features

- **RPS-976**: Added integration test CI (#25)
  ([#25](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/25),
  [`15fc353`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/15fc35340c485ba6ceda241039bff5a468018b24))


## v0.14.0 (2025-01-20)

### Features

- Add Auth0 device code authorization and refresh token flow (#31)
  ([#31](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/31),
  [`06f42ab`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/06f42ab2f22511ad7c8ed6fc4ad673008b9f49e9))


## v0.13.0 (2025-01-20)

### Features

- Expose virtual robot api (#30) ([#30](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/30),
  [`a43f73a`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a43f73aa055aae2811e93d9f5dff6a8f9c8511f1))


## v0.12.0 (2025-01-16)

### Features

- Expose store collision scene api (#28)
  ([#28](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/28),
  [`d92b4dc`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d92b4dcda1e56246230d2ad99fb502cb3da755f2))


## v0.11.0 (2025-01-16)

### Features

- Add support for motion setting (#23)
  ([#23](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/23),
  [`dbbe8a6`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/dbbe8a637608b5ef2e649739c353b63a84169ebc))


## v0.10.1 (2025-01-15)

### Bug Fixes

- **RPS-1086**: Add option to close the nova object (#27)
  ([#27](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/27),
  [`e30eabc`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e30eabc9fb35e07e9def4fc52e29b1e3aa60ed24))


## v0.10.0 (2025-01-15)

### Features

- Expose store collision components api (#26)
  ([#26](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/26),
  [`0bfd51e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/0bfd51e5476b7c91d456380bc6d318c30caf50e7))


## v0.9.1 (2025-01-14)

### Chores

- Updated readme (#24) ([#24](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/24),
  [`fe17057`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/fe170573ecfabb06df46fae11ebf7905af7f9b0e))


## v0.9.0 (2025-01-13)

### Chores

- Add mypy checks to pre-commit
  ([`6dc3601`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6dc36015d76a85397640dfc136e33b9a043ecfed))

- Add pre-commit hook to sort imports
  ([`1a190f5`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1a190f5ed87d05082f883d081645017af3ede72f))

- Add yamllint to pre-commit
  ([`aa552fe`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/aa552fe121f81d45674be13903f61b54c782010d))

- Run isort across the project
  ([`a4a0a31`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a4a0a31c3a159cf39926182f649b41718f095508))

### Features

- Extend `Vector3d`
  ([`59e813c`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/59e813c71ce191f67c266e8887dbffeebf819d62))


## v0.8.0 (2025-01-09)

### Features

- Add flag to control ssl and improve the logging (#20)
  ([#20](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/20),
  [`e302fa7`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e302fa79f375e9ec1616c5a1e2b5ede9347bb7e3))


## v0.7.0 (2025-01-09)

### Features

- Add documentation to the examples (#21)
  ([#21](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/21),
  [`6b43084`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/6b4308407e2eb0f03ad6321e829f8ce6d5abe7ec))


## v0.6.1 (2025-01-08)

### Bug Fixes

- Provide autogenerated API from nova.api (#19)
  ([#19](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/19),
  [`d84a842`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/d84a842c975c5d4e890f1c65b88fbc9b8610018b))


## v0.6.0 (2025-01-07)

### Features

- Added .activated_motion_groups to fetch all motion groups from con (#18)
  ([#18](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/18),
  [`beac2e3`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/beac2e3a45b2143c6ed9fe4a2b1fd810b5f11b0e))


## v0.5.0 (2025-01-06)

### Features

- **RPS-1034**: Added mypy check to CI * updated API * added pose transform (#17)
  ([#17](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/17),
  [`8838bbf`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8838bbf4b60ab5475ef72a48700384b33a43beda))


## v0.4.0 (2024-12-23)

### Features

- **RPS-1027**: Separated plan and execute in motion group (#16)
  ([#16](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/16),
  [`69da340`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/69da34055c00ab5c5917b6e32e1c2fa6e8f9dc16))


## v0.3.0 (2024-12-23)

### Features

- **RPS-1004**: Handle https, add some convenience methods (#15)
  ([#15](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/15),
  [`e46cc90`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e46cc901923d290a99a06cc8fc5fa08ff6ee4502))


## v0.2.3 (2024-12-20)

### Chores

- Added yamllint (#14) ([#14](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/14),
  [`4c9b66e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/4c9b66eea27e25666d09d5ab70352988edacb8d4))


## v0.2.2 (2024-12-19)

### Chores

- Make motion_group plan call public (#13)
  ([#13](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/13),
  [`8c464ba`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/8c464ba5c3c05f59b68bbb0458db11f3e2422c97))


## v0.2.1 (2024-12-19)

### Chores

- Updated nova interface and simplified examples (#12)
  ([#12](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/12),
  [`2e096bb`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/2e096bb326094345bd6c30f922a5593ca46224e1))


## v0.2.0 (2024-12-17)

### Features

- **RPS-999**: Activate motion groups seperatly (#11)
  ([#11](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/11),
  [`e7a77ba`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/e7a77baaa9fed33f151e48103b7bc0f5d3b04c58))


## v0.1.9 (2024-12-13)

### Chores

- Add pre-commit (#10) ([#10](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/10),
  [`44e7b6d`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/44e7b6de2a9dd179ce1ea9afa1277cc077cee497))


## v0.1.8 (2024-12-12)

### Bug Fixes

- Not always use "Flange" as a parameter for tcp in _load_planned_motion (#9)
  ([#9](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/9),
  [`9eb7ef1`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9eb7ef1ccd3a7c21618c8cf3c3c156928606299a))


## v0.1.7 (2024-12-11)

### Chores

- Don't publish pi from nova package (#8)
  ([#8](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/8),
  [`32f7404`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/32f740427b95abeed7ff63564a5912fb0ed17f37))


## v0.1.6 (2024-12-10)

### Chores

- Add examples and refactoring (#5) ([#5](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/5),
  [`66f8a6e`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/66f8a6e71d2069ccc754f56c97a928da55d18598))

- Add license file (#6) ([#6](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/6),
  [`dc9401b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/dc9401bfa5bd2239feb612f71b122e2eb59cd452))

- Updated README and env variable handling (#7)
  ([#7](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/7),
  [`a30cfd3`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a30cfd34e4d36e78316b37b1e8dd61ce0cb47f1e))


## v0.1.5 (2024-12-06)

### Chores

- Updated examples link
  ([`9d0b858`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/9d0b8589bf0ee34438cf6b39b8e519080fe622e1))


## v0.1.4 (2024-12-06)

### Chores

- Try wandelbots-nova for pypi upload
  ([`a583702`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/a5837020d9c2640e5646ec78cdf269d13c1fadb8))


## v0.1.3 (2024-12-06)

### Chores

- Try nova-python for pypi upload
  ([`09fa372`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/09fa3726ac256d0a9e26dcaa4f1bbaa6da956101))


## v0.1.2 (2024-12-06)

### Chores

- Try novapy for pypi upload
  ([`f93107b`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/f93107b49a7cb1eb48ddb47cc936f5058bbaca36))


## v0.1.1 (2024-12-06)

### Chores

- Updated README.md
  ([`1ab8485`](https://github.com/wandelbotsgmbh/wandelbots-nova/commit/1ab8485d451bdf9cb55d3706a4d62cd916b25442))


## v0.1.0 (2024-12-06)

- Initial Release
