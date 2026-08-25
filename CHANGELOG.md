# Changelog

All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this project adheres to [Semantic Versioning](https://semver.org/).

## [1.5.0](https://github.com/FGH-wirtz/dpsim/compare/v1.4.0...v1.5.0) (2026-08-25)


### Added

* add --WithHydroGovernor flag to SP_ReducedOrderSG_SMIB_Fault example ([3a2807a](https://github.com/FGH-wirtz/dpsim/commit/3a2807aa8a5b8e5702541a54ea82eb95d063d91c))
* add configurable PFSolver base-power fallback and fix dead max-iterations setter ([396ed07](https://github.com/FGH-wirtz/dpsim/commit/396ed07570998fb30dd46571ca6cb24aa7bf7e70))
* add createSubComponents() hook to MNAInterface for early subcomponent registration ([d6941dc](https://github.com/FGH-wirtz/dpsim/commit/d6941dc5f7dc0cc23e6b8e4f2b2d525938989fe5))
* add DP generalized SSN RLC validation notebook ([e8e1397](https://github.com/FGH-wirtz/dpsim/commit/e8e1397f6cd3e17641460b02ce399569a20566bf))
* add DP generic two-terminal SSN example ([6bd7fe7](https://github.com/FGH-wirtz/dpsim/commit/6bd7fe7b330f287201e2b6dd2621383fd66bc1df))
* add DP Ph1 components: Two-Terminal and Serial RLC ([869adca](https://github.com/FGH-wirtz/dpsim/commit/869adca448ed77426622aef04d5bac9d587901ef))
* add DP Ph1 generic two-terminal V/I-type SSN components ([10b4858](https://github.com/FGH-wirtz/dpsim/commit/10b4858b46e6784a4801d14bfac183232d142df5))
* add DP serial RLC SSN example ([573c629](https://github.com/FGH-wirtz/dpsim/commit/573c6290b9d33c3fce3e7e4a85ba609c87108c97))
* add DP SSN base class ([c00b93a](https://github.com/FGH-wirtz/dpsim/commit/c00b93a878153696c73d51c5d7659861d5c2d62e))
* add DP SSN RLC accuracy sweep notebook ([0146efc](https://github.com/FGH-wirtz/dpsim/commit/0146efcc7dc179337c93216dc6f999b87bd081de))
* add DP V-type and I-type SSN bases ([316fcfd](https://github.com/FGH-wirtz/dpsim/commit/316fcfd9ac36eb509198b7a58c60e8907c756d9d))
* add DP_Ph3 CurrentSource, fix per-phase source signals ([909b829](https://github.com/FGH-wirtz/dpsim/commit/909b82953a4bda4eded6c9eab58c0a42a7488ac2))
* add DP_Ph3 SSN base and generic components ([41442ab](https://github.com/FGH-wirtz/dpsim/commit/41442ab94a11a26331fddb928fd12414e1600a84))
* add DP_Ph3 SSN base and generic components with examples ([#541](https://github.com/FGH-wirtz/dpsim/issues/541)) ([2280f11](https://github.com/FGH-wirtz/dpsim/commit/2280f1179351754d324c66b724ad94c4386c5de3))
* add ExciterModels SMIB fault notebook with all 4 exciters and PSS Kp/Kv test ([43ffa30](https://github.com/FGH-wirtz/dpsim/commit/43ffa30c7ca83555dbc79f92ebe4cd79d49d1469))
* add getBaseVoltage() getter to SP::Ph1::Shunt ([93e5b46](https://github.com/FGH-wirtz/dpsim/commit/93e5b46e73e2074a967a9417e18e527121d2899f))
* add HydroTurbineGovernor and HydroTurbine signal models ([c374230](https://github.com/FGH-wirtz/dpsim/commit/c37423028de098ed74f4929289d792e50cc93242))
* add opt-in filter for out-of-service equipment and isolated buses in matpower reader ([dcb0722](https://github.com/FGH-wirtz/dpsim/commit/dcb072228ed0a6d695998ad496a41f2ae55c899f))
* Add option to map synchronous machine to ideal voltage source in matpower ([#601](https://github.com/FGH-wirtz/dpsim/issues/601)) ([47b236e](https://github.com/FGH-wirtz/dpsim/commit/47b236e4a0a8473edc4d7f6f6fa7a5fab744b8e9))
* add Python binding for DP serial RLC SSN ([bccb426](https://github.com/FGH-wirtz/dpsim/commit/bccb426ad6fce0609beb0e9d31efbf51a6d7bb0f))
* add Python bindings ([7862042](https://github.com/FGH-wirtz/dpsim/commit/7862042b41739c5e74ba6cb84958f8434825eb22))
* add Python bindings for DP generic two-terminal SSN ([67171b6](https://github.com/FGH-wirtz/dpsim/commit/67171b66d16d30e353cc4c2d4ee0d5c48f956456))
* add SP/DP/EMT validation notebook for all SG controllers ([ff7c1b4](https://github.com/FGH-wirtz/dpsim/commit/ff7c1b41440854b2484d3620bfd9055dcea9f3a2))
* add SteamTurbineGovernor and SteamTurbine signal models ([e7d3bac](https://github.com/FGH-wirtz/dpsim/commit/e7d3bac1af460c6477bcffc007f4ea2b5954577d))
* Added Grid-forming Inverter, VCO and VoltageController ([#560](https://github.com/FGH-wirtz/dpsim/issues/560)) ([6f66436](https://github.com/FGH-wirtz/dpsim/commit/6f66436331c24f92282be9093b5715dabcc8c795))
* auto-generate .pyi stub files for dpsimpy and dpsimpyvillas ([6474dd2](https://github.com/FGH-wirtz/dpsim/commit/6474dd29434166f8ef16350a8e03dd8c4f1606e5))
* configurable PFSolver base-power fallback and working max-iterations setter ([#552](https://github.com/FGH-wirtz/dpsim/issues/552)) ([8d98959](https://github.com/FGH-wirtz/dpsim/commit/8d989596c001f7948db096fe1e49bf29e3555e73))
* **DP:** add DP::Ph3::NetworkInjection three-phase slack ([7fdd907](https://github.com/FGH-wirtz/dpsim/commit/7fdd9074f6c71f5df9fe345d4080bfddf85d5ab6))
* **DP:** add DP::Ph3::Switch three-phase matrix switch ([54ca8bd](https://github.com/FGH-wirtz/dpsim/commit/54ca8bd9d9a32319a4e8450779f531af7e440fad))
* EMT Ph3 SSN grid-forming inverter + automatic GFM controller derivation + CIM reader robustness ([#570](https://github.com/FGH-wirtz/dpsim/issues/570)) ([30eb7ea](https://github.com/FGH-wirtz/dpsim/commit/30eb7ea484142c0ddd39fabc5a7570521f11bef2))
* **examples:** add DP Ph3 inverter modal comparison ([f1065f4](https://github.com/FGH-wirtz/dpsim/commit/f1065f4fff0130840575b989f5cc05cbd94d5375))
* expand test sections, add zoom widget ([5ac4d74](https://github.com/FGH-wirtz/dpsim/commit/5ac4d746416547bf440979ac9a7a24d10dd6f418))
* **gfl:** add EMT GFL models and SSN variants ([605853c](https://github.com/FGH-wirtz/dpsim/commit/605853c70cfed13bccea4eacef1e42f0a2263d16))
* integrate in SG and add Python bindings ([04f5710](https://github.com/FGH-wirtz/dpsim/commit/04f57100e00b4f0bc07075e4f8de1240b2eb1c0a))
* integrate in SG base classes ([444befb](https://github.com/FGH-wirtz/dpsim/commit/444befb9ffcb5cada94d6ed25fe493547f459bed))
* integrate TurbineGovernorType1 with SynchronGenerator and add Python bindings ([#490](https://github.com/FGH-wirtz/dpsim/issues/490)) ([b479465](https://github.com/FGH-wirtz/dpsim/commit/b479465a208528b86619eba6476cac57b0e79732))
* **models:** add an EMT::Ph3::SSN::InductionMotor with a test circuit ([fc512e4](https://github.com/FGH-wirtz/dpsim/commit/fc512e46c8b542cd049921459587d8cfea33d0e6))
* **models:** add EMT Ph3 PQLoad ([e088744](https://github.com/FGH-wirtz/dpsim/commit/e0887442bed1dd9831e17585b87a18bdfcb843ed))
* **models:** add more logging for motor ([6d5a86a](https://github.com/FGH-wirtz/dpsim/commit/6d5a86a7463deb7243c7d18298d1e84d5e14672f))
* **models:** add switch modes to EMT and DP switches ([efde609](https://github.com/FGH-wirtz/dpsim/commit/efde609f2de09f51a85da627df5b8c5a406817f3))
* **models:** add switching modes to DP::Ph1::Switch ([a5c983a](https://github.com/FGH-wirtz/dpsim/commit/a5c983a64f4097851aecd0c8b36d0b53f4f3485d))
* **models:** add the EMT::Ph3 half decoupling line and its test example ([b9dce6c](https://github.com/FGH-wirtz/dpsim/commit/b9dce6cafd99ac9be0d271bce4ed2696bfcadc9e))
* **models:** give the EMT_Ph3 half decoupling line its travel-time delay ([0789116](https://github.com/FGH-wirtz/dpsim/commit/07891162af8d26f2bd7e6da06c0fb16a9116664f))
* **models:** implement the half decoupling line initialisation and step ([f8c5cb7](https://github.com/FGH-wirtz/dpsim/commit/f8c5cb79d39c3d89448bbd815b0e431237fe7b8d))
* **models:** ramp the resistance on closing in exponential ZCS mode ([e7b59bf](https://github.com/FGH-wirtz/dpsim/commit/e7b59bf0df4206ba1fa8740dd21998e0dd05b75c))
* opt-in filter for out-of-service equipment and isolated buses in matpower reader ([#553](https://github.com/FGH-wirtz/dpsim/issues/553)) ([d793d9a](https://github.com/FGH-wirtz/dpsim/commit/d793d9af83eaddd9e6e5c363a5dfbb31f819e8ee))
* **pybind:** expose the EMT::Ph3 half decoupling line to Python ([337b36d](https://github.com/FGH-wirtz/dpsim/commit/337b36d7d12299693c305fdbf0113e239311a359))
* restore EnergyConsumer SSH-derived P/Q reading in CIM reader ([916789f](https://github.com/FGH-wirtz/dpsim/commit/916789fe74127c2fd81a4f6975caca76e85586a0))
* restore EnergyConsumer SSH-derived P/Q reading in CIM reader ([#554](https://github.com/FGH-wirtz/dpsim/issues/554)) ([1d6ec86](https://github.com/FGH-wirtz/dpsim/commit/1d6ec86894b3530780f7ab058f9aec6b16c2dead))
* sparse-Jacobian powerflow solver with KLU refactorization reuse ([415e95a](https://github.com/FGH-wirtz/dpsim/commit/415e95aa3af690194192ea55baad2a06b40709a8))
* sparse-Jacobian powerflow solver with KLU refactorization reuse for large grids ([#514](https://github.com/FGH-wirtz/dpsim/issues/514)) ([5ff531e](https://github.com/FGH-wirtz/dpsim/commit/5ff531ee78a94438ae7bfcf479bc035b8852b258))
* **statespace:** support DP Ph3 state-space extraction ([b8ecb8a](https://github.com/FGH-wirtz/dpsim/commit/b8ecb8af76e4e808571636a500bdddc69d6689e0))
* **statespace:** support split SSN contributors ([58a2253](https://github.com/FGH-wirtz/dpsim/commit/58a2253329f7fb82061bc9074cba172d42df5f29))


### Changed

* academic tone and references for DP::Ph3 AvVSI state-space model ([c1fedff](https://github.com/FGH-wirtz/dpsim/commit/c1fedffc613fa0c82c5f678103cf8d8f6af14c31))
* add composite lifecycle regression notebook ([db0aa78](https://github.com/FGH-wirtz/dpsim/commit/db0aa7886da3e985dcbabdde9a8e2151deaeb240))
* add CONTRIBUTING.md and a notebook output compliance check ([#509](https://github.com/FGH-wirtz/dpsim/issues/509)) ([acaacf7](https://github.com/FGH-wirtz/dpsim/commit/acaacf7da56cc0ce016d8c48dfce2290fcf97789))
* add CONTRIBUTING.md and CI compliance checks ([262326e](https://github.com/FGH-wirtz/dpsim/commit/262326e66e066543f296e163441816bac80902bf))
* add documentation for Hydro Turbine Governor ([bae4630](https://github.com/FGH-wirtz/dpsim/commit/bae4630e43d5539a1beffb98eb9ce4671d41c8a2))
* Add documentation of Steam Governor and Steam Turbine ([cfab86b](https://github.com/FGH-wirtz/dpsim/commit/cfab86ba9dba427028bfee295282656521071e8c))
* add DP_Ph1 vs EMT_Ph3 averaged-VSI cross-domain validation notebook ([b554b93](https://github.com/FGH-wirtz/dpsim/commit/b554b93b38500d6cf1def28205c7d7fc6148dd33))
* add grid-forming SSN inverter model page ([9d7b0df](https://github.com/FGH-wirtz/dpsim/commit/9d7b0dff619eec2e0e86c09c1f3b32db63647586))
* add multi-stage LLM PR reviewer ([8134873](https://github.com/FGH-wirtz/dpsim/commit/8134873eb840ed9b218649693a44e2a7cf59f928))
* add multi-stage LLM PR reviewer ([#594](https://github.com/FGH-wirtz/dpsim/issues/594)) ([85ed0e6](https://github.com/FGH-wirtz/dpsim/commit/85ed0e6787b25fd62a1c1142444c86a94d16fc4c))
* add Q-limit PV&lt;-&gt;PQ switching notebook ([49ed3ed](https://github.com/FGH-wirtz/dpsim/commit/49ed3ed0930ebb835a41acb063d884a1380bd8df))
* add scripts that check coverage, pairing and unmarked hazards ([55993ce](https://github.com/FGH-wirtz/dpsim/commit/55993ce82a7731988953ff4a3fca31b361ae0e78))
* add SPDX header to set_dev_version.py ([37521c2](https://github.com/FGH-wirtz/dpsim/commit/37521c2bf7a7a16a8f6ff02b602958470dc198e0))
* add State-Space Nodal in dynamic phasors concept page ([4cec11e](https://github.com/FGH-wirtz/dpsim/commit/4cec11e6d775aed33edb9438a451f44dba0a959b))
* add the citation SPDX header and fix the MathJax CDN path ([d7e516c](https://github.com/FGH-wirtz/dpsim/commit/d7e516c56da3d9dfc1dc142bbfe63f415be93c86))
* address scheduling page review and align architecture page ([faac682](https://github.com/FGH-wirtz/dpsim/commit/faac682691f902c1c88d0912d3c40fdcd12caa8d))
* build and publish Windows wheels ([8926096](https://github.com/FGH-wirtz/dpsim/commit/8926096d46a0171e2e4d9aaabae5284a7320ba6b))
* build images from fork pull requests behind an approval gate ([f6112cb](https://github.com/FGH-wirtz/dpsim/commit/f6112cb32e4f642ce8b3d87e6f1d4e1879e7a05c))
* bump patch + dev suffix for Test PyPI publishes ([135ef54](https://github.com/FGH-wirtz/dpsim/commit/135ef54b46e8af294ec006028ae34e7b6a371d14))
* bundle Windows DLL dependencies and import-test dpsimpyvillas on Linux ([f600d2e](https://github.com/FGH-wirtz/dpsim/commit/f600d2eeacdde61d9623869d7a45f26eb51b5912))
* cache the container images in the registry instead of the Actions cache ([4133223](https://github.com/FGH-wirtz/dpsim/commit/4133223af07a15a54a051cfdd5f9c10b63d832aa))
* cancel superseded workflow runs via concurrency groups ([251808f](https://github.com/FGH-wirtz/dpsim/commit/251808fb618b94216993bc52b1c6f8815a0baa92))
* cancel superseded workflow runs via concurrency groups ([#534](https://github.com/FGH-wirtz/dpsim/issues/534)) ([1406663](https://github.com/FGH-wirtz/dpsim/commit/14066639fd1a89fc9c8c3926a9b469b78fae6d5e))
* carry the uv extras check and the dev-vscode image into the restructure ([721c591](https://github.com/FGH-wirtz/dpsim/commit/721c591c18b3a5f76e1679f8489d0e991fa07dfd))
* **ci:** strip trailing whitespace in the container workflow ([9058c2f](https://github.com/FGH-wirtz/dpsim/commit/9058c2f879782b286fbbcdc24389b58998f10c9a))
* **codeowners:** add [@georgii-tishenin](https://github.com/georgii-tishenin) on the catch-all and the SSN models ([d2a878d](https://github.com/FGH-wirtz/dpsim/commit/d2a878de85ec761c20de4afdcf0fe0dd7bb5fb20))
* **codeowners:** add [@leonardocarreras](https://github.com/leonardocarreras) to the catch-all ([b22b8f0](https://github.com/FGH-wirtz/dpsim/commit/b22b8f04d63fcaa3a8a5cebb8e774bec126244ae))
* **codeowners:** add [@matthiasmees](https://github.com/matthiasmees) on the components he was involved in ([3182a3f](https://github.com/FGH-wirtz/dpsim/commit/3182a3f5a641052db7bb236a67e311ba8bd43a04))
* **codeowners:** keep [@stv0g](https://github.com/stv0g) only on VILLAS and Nix on request ([8a9fdf5](https://github.com/FGH-wirtz/dpsim/commit/8a9fdf5160712572a0ea19d724e7199594a33ee5))
* **codeowners:** remove [@m-mirz](https://github.com/m-mirz) on request ([c306944](https://github.com/FGH-wirtz/dpsim/commit/c3069447789225e705c4fcf79ce8a12c3b7a73ee))
* **codeowners:** repoint the Utils entries and drop the deleted requirements file ([eae42d4](https://github.com/FGH-wirtz/dpsim/commit/eae42d4e2e1f1f7de0ac194862524a16f58735ac))
* **conventions:** document DP Ph3 peak-envelope scaling ([ff2529e](https://github.com/FGH-wirtz/dpsim/commit/ff2529e1e1cbf75e3a7b67bed0a320a571120be7))
* Define generator model as an enum. ([40e8d64](https://github.com/FGH-wirtz/dpsim/commit/40e8d64137c383578a99a38c38219c4302280fd3))
* describe the continuous integration workflow layout ([e3ea2ba](https://github.com/FGH-wirtz/dpsim/commit/e3ea2baa8bdf5c3713f89f59c72b9716ea48fce4))
* describe the Windows release build flags ([d869d71](https://github.com/FGH-wirtz/dpsim/commit/d869d712d94f58d9d43a5f528613fcacc9052569))
* document degenerate-value guard and name composite lifecycle stages ([ade794a](https://github.com/FGH-wirtz/dpsim/commit/ade794a5ded680e468cea9b6876040239f9684b7))
* document DP::Ph1 averaged-VSI state-space inverter ([338b0b8](https://github.com/FGH-wirtz/dpsim/commit/338b0b8b000338172b5aeb60349eef843dd7c946))
* document DP::Ph1 averaged-VSI state-space inverter ([#549](https://github.com/FGH-wirtz/dpsim/issues/549)) ([575952c](https://github.com/FGH-wirtz/dpsim/commit/575952c49de61c8abe02ea86add09362dcc86321)), closes [#547](https://github.com/FGH-wirtz/dpsim/issues/547)
* document generator reactive power limit enforcement ([72c454f](https://github.com/FGH-wirtz/dpsim/commit/72c454ff7baf5e736d2e4b8cad8688839aaa2f97))
* document MNA solver and component initialization ([#505](https://github.com/FGH-wirtz/dpsim/issues/505)) ([35752bf](https://github.com/FGH-wirtz/dpsim/commit/35752bfe528c6280d9b653f021c6b3ac96aa29cf)), closes [#59](https://github.com/FGH-wirtz/dpsim/issues/59)
* document MNA solver and component initialization (closes [#59](https://github.com/FGH-wirtz/dpsim/issues/59)) ([31a1d6e](https://github.com/FGH-wirtz/dpsim/commit/31a1d6ec5325b13a16735e0f9d498b068c03a239))
* document two-phase composite initialization lifecycle ([87e27d2](https://github.com/FGH-wirtz/dpsim/commit/87e27d2730ba07bbd8d7857d4725c5f203c2e8f2))
* drop the merge-commit checks and harden the notebook compliance job ([7c1a084](https://github.com/FGH-wirtz/dpsim/commit/7c1a0846b97092b15982e4f04636c1e06ee83ec5))
* drop the merge-commit checks and harden the notebook compliance job ([e33a892](https://github.com/FGH-wirtz/dpsim/commit/e33a8926a29c649a626e1fac96d299be4ab46871))
* enforce conventional commits for commit messages ([b6f1851](https://github.com/FGH-wirtz/dpsim/commit/b6f1851e66a229a11030c320747a4cd3aaaf0efe))
* **examples:** skip WSCC 9-bus switch notebook while its reference URL is dead ([9370bae](https://github.com/FGH-wirtz/dpsim/commit/9370bae1c0875564f801470db4d2e9a3f661d9dd))
* **examples:** update the half decoupling line test example ([e8ac690](https://github.com/FGH-wirtz/dpsim/commit/e8ac690810db12f81ed522ec57d2950da5cb661e))
* expand Scheduling page with concept overview and developer guide ([1991d0a](https://github.com/FGH-wirtz/dpsim/commit/1991d0a500030bdf5988ada90958700f4eae06dd))
* expand task scheduling page ([#188](https://github.com/FGH-wirtz/dpsim/issues/188)) ([#506](https://github.com/FGH-wirtz/dpsim/issues/506)) ([15ef5d9](https://github.com/FGH-wirtz/dpsim/commit/15ef5d9a5c7c143dfa7538426d15a7d4c3566280))
* fix PSS1A equations to include Kp/Kv input signals, add note about the figure ([c40f8b1](https://github.com/FGH-wirtz/dpsim/commit/c40f8b1fd69d736168c4d52ab517a1aa77dc830c))
* fix Sonar scan metadata resolution for fork PRs ([#598](https://github.com/FGH-wirtz/dpsim/issues/598)) ([b8a6e15](https://github.com/FGH-wirtz/dpsim/commit/b8a6e15aee260bef09c0b9e3d7d7ce1d27fff52f))
* fix the site configuration and publish the API reference ([e1059e5](https://github.com/FGH-wirtz/dpsim/commit/e1059e5c01794b7ac1a05d80fa45d98fc4d127bc))
* fix typo in the transfer function numerator of HydroTurbine ([19eda79](https://github.com/FGH-wirtz/dpsim/commit/19eda797b53c2fcca1f9fc7aedb7ba2dcaa8c982))
* give the reusable workflow jobs readable step names ([4971234](https://github.com/FGH-wirtz/dpsim/commit/4971234fcb9d345d3a4c224e6fdc459db2829cfc))
* ground LLM reviewer in real code with verification, PR-intent/claim-check, and safeguards ([31dc024](https://github.com/FGH-wirtz/dpsim/commit/31dc02435dcaf59c3a9c190b442e1e000f8a8d97))
* hoist mSubCompCreated into CompositePowerComp base class ([83255e0](https://github.com/FGH-wirtz/dpsim/commit/83255e0eb02d9c1f35d4c37e25a07e5359ef3272))
* **hugo:** add mermaid shortcode and body-end hook ([0d735ea](https://github.com/FGH-wirtz/dpsim/commit/0d735eac2d5c58799e1c08fdcb6d5deba6bb9be6))
* improve language and clarify initializeParentFromNodesAndTerminals ([959971a](https://github.com/FGH-wirtz/dpsim/commit/959971aa63314b68b638cc985e6976e6b28de8ef))
* install ccache in the Fedora development images ([3d5419b](https://github.com/FGH-wirtz/dpsim/commit/3d5419b21074915f60aeb3842606055c5fbe5bf9))
* **install:** add the supported version matrix and the windows wheel limits ([1fc6854](https://github.com/FGH-wirtz/dpsim/commit/1fc6854795c5df31e39606a8348cb25b65ad187d))
* key concurrency group on PR number instead of head_ref ([8b8d39c](https://github.com/FGH-wirtz/dpsim/commit/8b8d39c198b43fc3abc12c2f396c7003dfb2df4d))
* LLM reviewer handles oversized diffs and unresolvable inline lines ([51e922e](https://github.com/FGH-wirtz/dpsim/commit/51e922ef6e83d9f98aee1ace368c7d7ecfdb0547))
* LLM reviewer handles oversized diffs and unresolvable inline lines ([#597](https://github.com/FGH-wirtz/dpsim/issues/597)) ([500ba56](https://github.com/FGH-wirtz/dpsim/commit/500ba56a66dc37f5daf5694ba36f7c78ffa7157a))
* LLM reviewer verify stage keeps missing-SPDX findings ([#596](https://github.com/FGH-wirtz/dpsim/issues/596)) ([fc6a743](https://github.com/FGH-wirtz/dpsim/commit/fc6a7438531e1f430917d8ca40ca9e5c83889676))
* **llm-review:** stop posting tentative findings ([f4d3412](https://github.com/FGH-wirtz/dpsim/commit/f4d34129c1a0817f6b6009c2e63eac75299b51f9))
* **llm-review:** stop repeating tentative findings on every push ([01b32de](https://github.com/FGH-wirtz/dpsim/commit/01b32dee232ce84b84f017a73a1acec4106d20f9))
* manage releases and version bumps with release-please ([29f1693](https://github.com/FGH-wirtz/dpsim/commit/29f16937d2e469d0807bf9f9e494e534c6de2e64))
* **master:** release 1.4.0 ([cc5ffe7](https://github.com/FGH-wirtz/dpsim/commit/cc5ffe7bde07a18c621861884b2f6f8dc9ce93d8))
* migrate DP_Ph1_SVC to CompositePowerComp ([e7234bc](https://github.com/FGH-wirtz/dpsim/commit/e7234bc0d08c0b80ef09967b02bd6f1f9a317a16))
* **models:** add SPDX headers and CODEOWNERS entries for the half decoupling line ([95b9023](https://github.com/FGH-wirtz/dpsim/commit/95b90235f38ac3cb31f6d9c4304644ef222dcc40))
* **models:** clarify switch model description ([567fce8](https://github.com/FGH-wirtz/dpsim/commit/567fce83676466fb7480bb0ff877d32cdddf0928))
* **models:** document the EMT::Ph3 half decoupling line ([6628ac1](https://github.com/FGH-wirtz/dpsim/commit/6628ac1e0c31b9ee2b2891fc61cc2dca3255cec6))
* **models:** extract common split SSN component logic ([096d19e](https://github.com/FGH-wirtz/dpsim/commit/096d19e89da94fcb70a0b4dbfaf14f44876c7ad9))
* move the build jobs into their own reusable workflows ([5ea3b64](https://github.com/FGH-wirtz/dpsim/commit/5ea3b6408a43ca4841087494c14a7fbbd30e2291))
* normalize DP_Ph3_AvVoltSourceInverterStateSpace notebook formatting ([cb2d9be](https://github.com/FGH-wirtz/dpsim/commit/cb2d9befa3100640747fe6514722c2be5d7d1af2))
* note that wheels are published for Windows too ([e987812](https://github.com/FGH-wirtz/dpsim/commit/e98781256be3f327ae2a1598c5e19416ee32c871))
* note the LLM provenance on every page ([efa01c7](https://github.com/FGH-wirtz/dpsim/commit/efa01c7368bcb47852f7193a1ee00a20428c822a))
* note the per-image container rebuild rules ([c0ac8b4](https://github.com/FGH-wirtz/dpsim/commit/c0ac8b4c213363d7e5de6958f91d272fd1160eda))
* note the Windows wheel feature gaps and PowerShell venv steps ([9900538](https://github.com/FGH-wirtz/dpsim/commit/9900538741fad94d19d01af07ba1552a8896ceb3))
* **notebooks:** skip slow notebook tests in CI ([aae182c](https://github.com/FGH-wirtz/dpsim/commit/aae182cacfb96106d158ffa07c93c5c691ce6572))
* orchestrate all workflows from a single CI entry point ([0e93ac0](https://github.com/FGH-wirtz/dpsim/commit/0e93ac06f90e61fa862711339debf161f90a5b95))
* patch + dev suffix for Test PyPI publishes ([#565](https://github.com/FGH-wirtz/dpsim/issues/565)) ([ed2897f](https://github.com/FGH-wirtz/dpsim/commit/ed2897f9bc53297a03a07ab8ca6affa624466c1f))
* point the README badge and build guide at the CI entry point ([2b0be17](https://github.com/FGH-wirtz/dpsim/commit/2b0be17f622361f374db1ce292dd453fca270124))
* polish Turbine Governor Type 1 equations and wording ([3a7d0f1](https://github.com/FGH-wirtz/dpsim/commit/3a7d0f1b78703520559fbe302c803b0e42e969ec))
* publish to PyPI from a dedicated workflow ([a95961f](https://github.com/FGH-wirtz/dpsim/commit/a95961fc5ebf78e672a38028d713bfb6d8f58b4c))
* **python:** drop 3.9 and build wheels for 3.14 ([966ec80](https://github.com/FGH-wirtz/dpsim/commit/966ec8017cebcc49b5c50af65eea274a9566e8db))
* rebuild only the container images whose definition changed ([7e65fb7](https://github.com/FGH-wirtz/dpsim/commit/7e65fb797dfa02a70ac61da8793f04ecae2fb8ce))
* register in CMake and Factory ([8739d6a](https://github.com/FGH-wirtz/dpsim/commit/8739d6acb29320d08de3adf2900330f708fc2333))
* release 1.3.0 ([46901dd](https://github.com/FGH-wirtz/dpsim/commit/46901dd0a76e63cb864fc959d4987ac4a83de19b))
* remove commented-out dead code in PFSolverPowerPolar ([f20a2cd](https://github.com/FGH-wirtz/dpsim/commit/f20a2cd06456e62d779f46cd16957c2e74d0e2c2))
* remove duplicate initialize(mFrequencies) calls and migrate SVC to CompositePowerComp ([#508](https://github.com/FGH-wirtz/dpsim/issues/508)) ([#511](https://github.com/FGH-wirtz/dpsim/issues/511)) ([4d49d5b](https://github.com/FGH-wirtz/dpsim/commit/4d49d5b7e1f6ee088f81c48fa138a27dcd486287))
* remove duplicate initialize(mFrequencies) calls from createSubComponents ([6e6613b](https://github.com/FGH-wirtz/dpsim/commit/6e6613b9a8caf5d7213c5a373512f8c8ddda3395))
* rename CompositePowerComp subcomponent task lists for clarity ([e0865c2](https://github.com/FGH-wirtz/dpsim/commit/e0865c2ac4b8e025aea521afde68ae8b88a6db3b))
* rename initialize to initializeStates on Base::Exciter and Base::PSS ([4e19d25](https://github.com/FGH-wirtz/dpsim/commit/4e19d253ba6622e7408aa5cb1cf52c0e2c81d555))
* resolve fork PR metadata by head owner:branch and skip stale post-merge builds in Sonar scan ([ad07b7e](https://github.com/FGH-wirtz/dpsim/commit/ad07b7e63b226145df12ab52f010b145ddb8d812))
* resolve pyproject extras with uv instead of pip-tools ([641b048](https://github.com/FGH-wirtz/dpsim/commit/641b0483951c90e923c967079965928f96fa2fae))
* restore github hosted runner runs-on references ([f5bbd21](https://github.com/FGH-wirtz/dpsim/commit/f5bbd2175dea72b7d58b2d3f6df6ba525acaaa8a))
* restructure the documentation and close the coverage gaps ([95d2279](https://github.com/FGH-wirtz/dpsim/commit/95d2279854bfacade316ef23fc5c1c9fd2a37fb2))
* restructure the documentation and close the coverage gaps ([#619](https://github.com/FGH-wirtz/dpsim/issues/619)) ([64fcc98](https://github.com/FGH-wirtz/dpsim/commit/64fcc98bb178fe92888dda43dc1bed21f3887b7a))
* run fork pull requests through the same pipeline as the rest ([731b6a2](https://github.com/FGH-wirtz/dpsim/commit/731b6a2351ef80af4e0aa319e0b3b127bb46d18f))
* shared elementwise complex-product helper ([dee034c](https://github.com/FGH-wirtz/dpsim/commit/dee034c05690e4da3e77c0e29a1a4bd945b04bca))
* shared elementwise complex-product helper ([#584](https://github.com/FGH-wirtz/dpsim/issues/584)) ([a0173dd](https://github.com/FGH-wirtz/dpsim/commit/a0173dd8a9d442e2ed9644b9e5ede87aefeacdeb))
* skip case145/case300 CIM notebooks, bad upstream data ([bffb131](https://github.com/FGH-wirtz/dpsim/commit/bffb131028a7f6c28a36fe237fb54c0af13e3f05))
* **sonar:** scope cpp:S5184 out of the pybind bindings ([ac30249](https://github.com/FGH-wirtz/dpsim/commit/ac30249a573f05c8bbff2dd77543349153d16e40))
* split CompositePowerComp init into topology/parameterization hooks ([d4b35f1](https://github.com/FGH-wirtz/dpsim/commit/d4b35f15e1fd8346bbc7712a7a2de196e1342f1e))
* **ssn:** clarify Norton stamping for mixed DP models ([752b1a0](https://github.com/FGH-wirtz/dpsim/commit/752b1a03b3f77cf416ef32f44eb1614a65a7766d))
* strip notebook outputs via pre-commit and CI ([e33a892](https://github.com/FGH-wirtz/dpsim/commit/e33a8926a29c649a626e1fac96d299be4ab46871))
* strip saved outputs and backfill cell ids across all notebooks ([#605](https://github.com/FGH-wirtz/dpsim/issues/605)) ([9e864da](https://github.com/FGH-wirtz/dpsim/commit/9e864daa8458a08bb7d59bfe818faa51f41c88f7))
* tier LLM reviewer models across finders and escalating verify gates ([260720b](https://github.com/FGH-wirtz/dpsim/commit/260720bf8c6ce8660ae29445aecfc5a10f4ef2c2))
* tier LLM reviewer models across finders and escalating verify gates ([#595](https://github.com/FGH-wirtz/dpsim/issues/595)) ([bae4e48](https://github.com/FGH-wirtz/dpsim/commit/bae4e48110c4c5c445836de4c1cbddca82939b0b))
* tiered verify (uncapped mid, single-call final), serial finders with 429 backoff, temperature-400 retry, synthesis title guard ([5780c9f](https://github.com/FGH-wirtz/dpsim/commit/5780c9fa0d522ff7be5f78f9cf1b7350e2d6e91d))
* tighten composite comments and rename lifecycle phases to stages ([f6a075b](https://github.com/FGH-wirtz/dpsim/commit/f6a075b0f333538f95004eaf5bc177d3157811e6))
* verify stage confirms missing-SPDX findings instead of refuting them as policy ([dd6962a](https://github.com/FGH-wirtz/dpsim/commit/dd6962a00337ef2e5a324682601ae30535f022c5))


### Fixed

* add missing override specifiers for Clang ([e406768](https://github.com/FGH-wirtz/dpsim/commit/e406768cb13c7b41f942f7a3a521fd8e25a3ce70))
* add missing Shunt branch to PFSolver::determineNodeBaseVoltages() ([47f6e98](https://github.com/FGH-wirtz/dpsim/commit/47f6e9885dd5ae442b4327e8874f479245c2aaa1))
* add missing Shunt branch to PFSolver::determineNodeBaseVoltages() ([#551](https://github.com/FGH-wirtz/dpsim/issues/551)) ([d60d7f5](https://github.com/FGH-wirtz/dpsim/commit/d60d7f514afcb4e1abd66d258a78d420d9248efa))
* add numpy to build deps ([9716724](https://github.com/FGH-wirtz/dpsim/commit/9716724391077ccf0b6a4c579038d38268884903))
* avoid NaN from zero-capacitance PiLine shunt branches ([337b0e4](https://github.com/FGH-wirtz/dpsim/commit/337b0e49cf893443676c8c53390ef1a4f39ae8c7))
* Avoid override of signature for initialize(...)  function, renamed to initializeStates(...) ([c933640](https://github.com/FGH-wirtz/dpsim/commit/c933640db1f6672ac5b0a77023a3670c8c1aefac))
* **build:** pin the wheel build to the target interpreter for pybind11 ([93c1cbe](https://github.com/FGH-wirtz/dpsim/commit/93c1cbe3be5879ac6ea772ebe3cceb4e4ee0f41c))
* **ci:** build the manylinux container against a supported Python ([8e3212b](https://github.com/FGH-wirtz/dpsim/commit/8e3212b9be087b4fb7e0263e28c96fbe910d8a0c))
* **ci:** drop the Python dependency layer from the manylinux image ([65b66da](https://github.com/FGH-wirtz/dpsim/commit/65b66da6795a539f721c4549268591c778f318b2))
* **ci:** drop unused build tooling install from the manylinux venv ([9d69800](https://github.com/FGH-wirtz/dpsim/commit/9d69800f14b61350aff6172003104d9f52162433))
* **ci:** install libxil in the manylinux image from GitHub ([a48146e](https://github.com/FGH-wirtz/dpsim/commit/a48146e406eb2877820d3ef03eb8c5ffe4cf95bb))
* **ci:** install manylinux extras into a venv instead of the cp312 interpreter ([0c5cc9f](https://github.com/FGH-wirtz/dpsim/commit/0c5cc9ff5336db903f6f3b8696e65297631109d9))
* **ci:** let the approved fork gate check out fork pull requests ([d73f75f](https://github.com/FGH-wirtz/dpsim/commit/d73f75fb3928e1add20bf94baa599c137b51e25d))
* CIM reader PQ-generator mapping and slack voltage unit bug ([#558](https://github.com/FGH-wirtz/dpsim/issues/558)) ([83d14f5](https://github.com/FGH-wirtz/dpsim/commit/83d14f50006aae90f8c346a9ae03176d1650bc4d))
* **ci:** publish sogno/dpsim:dev-vscode and use it for the devcontainer ([90a7eca](https://github.com/FGH-wirtz/dpsim/commit/90a7ecaa2bc5aeb6e4784722c76fb7493b195238))
* **ci:** resolve Python dependencies with uv in the Rocky container ([d21d036](https://github.com/FGH-wirtz/dpsim/commit/d21d0361ae8c6332ef12e70f4c3def362e13674d))
* **ci:** stop building the images against the RWTH GitLab ([23dd85e](https://github.com/FGH-wirtz/dpsim/commit/23dd85e50e0c8febd78d7315efa782b4bdb7f825))
* **cmake:** stop passing GCC optimization flags to MSVC ([c82a66d](https://github.com/FGH-wirtz/dpsim/commit/c82a66da052fc358b498c026920a2709c30a3502))
* compute impedance in createSubComponents() before MNA pre-pass accesses it ([99e9391](https://github.com/FGH-wirtz/dpsim/commit/99e9391f7023d565d52fb9b034fc45d3f320a470))
* correct mVh_prev update order in PSS1A ([e54efe6](https://github.com/FGH-wirtz/dpsim/commit/e54efe659de512154a9781d234c43619ec46137c))
* correct N0.V filter in DP_CIGRE_MV_withDG_withLoadStep notebook ([590318f](https://github.com/FGH-wirtz/dpsim/commit/590318f2e3fb46a44cd7eec461d9bbe440144dc3))
* correct N0.V filter in DP_CIGRE_MV_withDG_withLoadStep notebook ([#510](https://github.com/FGH-wirtz/dpsim/issues/510)) ([2517003](https://github.com/FGH-wirtz/dpsim/commit/25170033d912aceffb8870e411d47936f86eb9c6))
* de-parameterize value-dependent composites ([e66c91a](https://github.com/FGH-wirtz/dpsim/commit/e66c91ad9c5be34917b046130f8572964577a578))
* **docker:** build libcimpp from source on ARM64/macOS ([64467d7](https://github.com/FGH-wirtz/dpsim/commit/64467d75ec7472074f33c0fc74080af8bd5e9760))
* **docker:** drop the spaces around the shmem image labels ([0eae7ad](https://github.com/FGH-wirtz/dpsim/commit/0eae7ad9fe5b47f455e197c0e9a5a0de5186ffde))
* DP_Ph1_AvVoltSourceInverterStateSpace power convention has no 3-phase multiplier ([dee01e3](https://github.com/FGH-wirtz/dpsim/commit/dee01e3b8280ffaa86558204c72f9cb1f9d751f3))
* **dp:** align DP Ph3 Series Resistor initialization scaling ([4602202](https://github.com/FGH-wirtz/dpsim/commit/4602202f4f3e5797f6d658065d8a628345719887))
* **dp:** correct Ph3 power-flow initialization scaling ([dfde80c](https://github.com/FGH-wirtz/dpsim/commit/dfde80cea209717ae1a91fde22397465ddda4ef0))
* **dp:** fix DP Ph3 mixed SSN Norton stamping and initialization ([d50ac8e](https://github.com/FGH-wirtz/dpsim/commit/d50ac8e3cdd627394ea3d1fc832495e0148c95f1))
* **dp:** preserve full Norton matrix for mixed Ph1 SSN components ([c25369c](https://github.com/FGH-wirtz/dpsim/commit/c25369c189aa5a9893f64332c182eb4d679718fa))
* **examples:** align DP Ph3 inverter checks with peak scaling ([e162b95](https://github.com/FGH-wirtz/dpsim/commit/e162b95cc0847bc46cf0a44508bbe9c55369b07a))
* exclude MNA fallback shunt from standalone PiLine PF stamp ([9e45630](https://github.com/FGH-wirtz/dpsim/commit/9e45630b57c86002c93a95e57137cf87a03044af))
* exclude MNA fallback shunt from standalone PiLine powerflow stamp ([#555](https://github.com/FGH-wirtz/dpsim/issues/555)) ([eaacc36](https://github.com/FGH-wirtz/dpsim/commit/eaacc365b2dcc9c4a69396847db1e43db06666d0))
* ExternalNetworkInjection voltage target unit disambiguation ([da27dee](https://github.com/FGH-wirtz/dpsim/commit/da27dee6e660b9f75a0b711f1eb55cdaf382b29a))
* ExternalNetworkInjection voltage target unit disambiguation ([#562](https://github.com/FGH-wirtz/dpsim/issues/562)) ([d42f2d9](https://github.com/FGH-wirtz/dpsim/commit/d42f2d9b676709902e6bdabf001a82dbbb7dd8f8))
* extract scalar gen/extnet values for matpower set_parameters ([6b1ce15](https://github.com/FGH-wirtz/dpsim/commit/6b1ce156c7a8a18f05062d394363cc0cc66d4606))
* **gfl:** address review comments ([bdce2e9](https://github.com/FGH-wirtz/dpsim/commit/bdce2e96d1300fd0cf3eaa4fbbbe80f2f8d69579))
* guard addPSS against null pointer ([777f59d](https://github.com/FGH-wirtz/dpsim/commit/777f59df6bf714b2aeddf690d9a47df29365c0e0))
* guard PF convergence check against non-finite mismatch ([855e347](https://github.com/FGH-wirtz/dpsim/commit/855e347a77dfa7d19378a4db291d6ceb0f047590))
* guard PF convergence check against non-finite mismatch ([#520](https://github.com/FGH-wirtz/dpsim/issues/520)) ([2732152](https://github.com/FGH-wirtz/dpsim/commit/27321529289579c08d22bde838b10a07378a1c17))
* guard R and T3 against non-positive values in SteamTurbineGovernor ([568a5b5](https://github.com/FGH-wirtz/dpsim/commit/568a5b5f667db3a5160fda457d346ed5ed9f6408))
* **models:** correct DP Ph3 capacitor initialization ([7eb721b](https://github.com/FGH-wirtz/dpsim/commit/7eb721bb35a8a81344ec5c05d11363bdcabb3938))
* **models:** correct DP Ph3 inductor resistor initialization ([2efee22](https://github.com/FGH-wirtz/dpsim/commit/2efee228721c70612a6fd2791625be0954e1324f))
* **models:** linearize the SSN grid-forming inverter analytically ([e8de609](https://github.com/FGH-wirtz/dpsim/commit/e8de6092ad61e064a100f2ed129a0e583091ebc7))
* **models:** move instead of forward the by-value attribute dependencies ([2cc345c](https://github.com/FGH-wirtz/dpsim/commit/2cc345c408d6994017d4ede9004a9d9f8220020b))
* **models:** zero-initialize the turbine governor parameters ([988d22e](https://github.com/FGH-wirtz/dpsim/commit/988d22efd7d6cdbf4a43e2b8ae888380ccfade2c))
* null guard on addGovernor and doc typo in TG Type1 equations ([cfff7fe](https://github.com/FGH-wirtz/dpsim/commit/cfff7fec58d9ddbf0020f63e501fb72f0fbc6b81))
* override createSubComponents() in all composite components ([#270](https://github.com/FGH-wirtz/dpsim/issues/270)) ([31a6b78](https://github.com/FGH-wirtz/dpsim/commit/31a6b78c6194600e9d55bb9029376d2667fe2f44))
* PF Newton-Raphson globalization + matpower reader/notebook CI fixes ([#512](https://github.com/FGH-wirtz/dpsim/issues/512)) ([79529ac](https://github.com/FGH-wirtz/dpsim/commit/79529aca2ea2a915b30d112cb39f4d71c9681f8a))
* PQ generators without RegulatingControl, ExternalNetworkInjection voltage unit bug ([da93680](https://github.com/FGH-wirtz/dpsim/commit/da936805e41513f9b24f5bfb601a802c398c8872))
* re-pivot fully when no variable matrix entries are declared ([aaa1659](https://github.com/FGH-wirtz/dpsim/commit/aaa1659948f21c0ed59445fd77d9b5c769e49c61))
* recurse into createSubComponents only for newly-registered subcomponents ([d64c147](https://github.com/FGH-wirtz/dpsim/commit/d64c147f5c37464c041866d75353d78995819094))
* refresh  matrix-node-index cache after terminal swap in SP Transformer PF ([6308bb6](https://github.com/FGH-wirtz/dpsim/commit/6308bb68ec118dcc85e912557c76375f79933069))
* refresh matrix-node-index cache after terminal swap in SP Transformer PF ([#521](https://github.com/FGH-wirtz/dpsim/issues/521)) ([533d066](https://github.com/FGH-wirtz/dpsim/commit/533d0663a6ade51684af0a422d56e06eb7871d35))
* remove duplicate getBaseVoltage() decl in SP_Ph1_Shunt ([8645a4d](https://github.com/FGH-wirtz/dpsim/commit/8645a4dcb4755239fb94b13d9ece38c0827dabae))
* rename HydroGorvernorParameters → HydroGovernorParameters in example ([8914a2b](https://github.com/FGH-wirtz/dpsim/commit/8914a2b0bc33ce39525223b073d99806cdc9e438))
* rename SteamGorvernorParameters → SteamGovernorParameters ([319c4ec](https://github.com/FGH-wirtz/dpsim/commit/319c4ec4b480f6fda1c69ef7d954944cdee6a982))
* replace PF Newton damping with whole-step scaling, restore 1e-8 tolerance ([01e603f](https://github.com/FGH-wirtz/dpsim/commit/01e603f562357607fc82455ccffb191adc7240f6))
* resolve Python dependencies with uv in the remaining images and scripts ([f336bb7](https://github.com/FGH-wirtz/dpsim/commit/f336bb7c908e43b0469d07fae57dfd2029a8958f))
* run stub generation once and raise on failure ([f6e5f97](https://github.com/FGH-wirtz/dpsim/commit/f6e5f976101446ef330805c260612adbb668cf3a))
* single ctor, rename HydroGovernorParameters, positive guards for R/T1/T3 ([91f968a](https://github.com/FGH-wirtz/dpsim/commit/91f968a865b25d05b8eb5d0dc9591879468067d4))
* SP::Ph1::Capacitor produces NaN admittance for zero capacitance ([#519](https://github.com/FGH-wirtz/dpsim/issues/519)) ([073599c](https://github.com/FGH-wirtz/dpsim/commit/073599c130d311271f77f3d38c15ec495986e551))
* SP::Ph1::Capacitor zero-capacitance NaN admittance ([28fce7b](https://github.com/FGH-wirtz/dpsim/commit/28fce7bf85e67b4c2426d04408469439afd993af))
* symmetric relative-error sweeps in DP SSN accuracy notebook ([678bf1f](https://github.com/FGH-wirtz/dpsim/commit/678bf1fd384a52d9014798f957eb484d86a313d3))
* **test:** update the pytest collect hook to the pytest 9 API ([136c655](https://github.com/FGH-wirtz/dpsim/commit/136c6550dd0a1409c10f97b759a0b73c8fda9b86))
* TurbineGovernorType1 initialization attributes ([#493](https://github.com/FGH-wirtz/dpsim/issues/493)) ([a844dfc](https://github.com/FGH-wirtz/dpsim/commit/a844dfca168f07609926d00f2aa407a37e2ede96))
* TurbineGovernorType1 quality improvements ([90c0105](https://github.com/FGH-wirtz/dpsim/commit/90c01056408702842492b42c58d54d7282033bfd))
* use correct SteamGovernorParameters and HydroGovernorParameters in notebook ([89162ec](https://github.com/FGH-wirtz/dpsim/commit/89162ecb80ecaf965b5393c8b72a1cad5244493f))
* use DOUBLE_EPSILON for exciter init consistency check; strip notebook outputs ([37a4147](https://github.com/FGH-wirtz/dpsim/commit/37a4147586e06f0a6c6a9b52c8ec11247029f672))
* use initializeStates for GovernorAndTurbine in Base_SynchronGenerator ([c3b13b2](https://github.com/FGH-wirtz/dpsim/commit/c3b13b2291ba6fdfa2da257f16be7207619ccf3f))
* use single constructor with default logLevel in TurbineGovernorType1 ([acf1c52](https://github.com/FGH-wirtz/dpsim/commit/acf1c52d7e3eab4a8a67681b43b450c74bd6b666))
* virtual node ordering for composite components ([#270](https://github.com/FGH-wirtz/dpsim/issues/270)) ([#496](https://github.com/FGH-wirtz/dpsim/issues/496)) ([e88667a](https://github.com/FGH-wirtz/dpsim/commit/e88667a29129a45c09fa01a6951ed8ee3e8629ee))
* zero-initialize PSS1AParameters fields and make state vars private ([31d36f8](https://github.com/FGH-wirtz/dpsim/commit/31d36f878eeb163795913dff8dfc42dd16ab387f))

## [1.4.0](https://github.com/sogno-platform/dpsim/compare/v1.3.0...v1.4.0) (2026-08-14)


### Added

* **models:** add an EMT::Ph3::SSN::InductionMotor with a test circuit ([fc512e4](https://github.com/sogno-platform/dpsim/commit/fc512e46c8b542cd049921459587d8cfea33d0e6))
* **models:** add more logging for motor ([6d5a86a](https://github.com/sogno-platform/dpsim/commit/6d5a86a7463deb7243c7d18298d1e84d5e14672f))


### Changed

* manage releases and version bumps with release-please ([29f1693](https://github.com/sogno-platform/dpsim/commit/29f16937d2e469d0807bf9f9e494e534c6de2e64))

## [v1.3.0] - 2026-08-14

### Added

- Generalized linear two terminal State Space Nodal component models (#406)
- Add generic EMT 3-phase two-terminal V-type SSN component (#472)
- Add variable SSN base and EMT 3-phase piecewise-linear inductor (#474)
- Add EMT::SSN ITypeSSNComps and classes for I-Type SSN (#489)
- Add FourTerminal_SSN classes (#500), also fixing the matpower.py import and the PF solver
- Generic Ph1 two-terminal linear SSN components for SP domain (#587)
- DP/SFA SSN: base classes and single-phase serial RLC component (#515)
- DP/SFA SSN: generic two-terminal V/I-type components (#516), adding single-phase validation and accuracy notebooks and a concept page for State-Space Nodal in dynamic phasors (#517)
- Add DP_Ph3 SSN base and generic components with examples (#541), adding notebooks, docs and base component pybind bindings (#542)
- DP: add Ph1 variable-SSN base for mixed real+complex state components (#544)
- Add DP::Ph3 mixed variable-SSN base (#578)
- Add automatic system matrix recomputation mode (#576)
- Add MNA state-space extraction and modal analysis (#478), supporting variable two-terminal SSN components (#488)
- Add GlobalDQ0 frame for state-space modal analysis (#497)
- Add state-space extraction for DP Ph1 components (#564) and for DP mixed variable SSN components (#571)
- Support EMT Ph3 and DP Ph1 composite components (#573) and switches (#574) in state-space extraction
- Add participation factors to state-space modal analysis (#537), adding right and left eigenvectors and named state metadata and exposing the modal participation results to Python
- Added Grid-forming Inverter, VCO and VoltageController (#560)
- EMT Ph3 SSN grid-forming inverter and automatic GFM controller derivation (#570), also making the CIM reader more robust
- Grid-connected grid-forming SSN inverter and IEEE 9-bus mixed example (#577)
- Feature: Added EMT classes GFL, refactored SSN_GFL and SSN_GFL_Split (#683), adding a common benchmark against the averaged-VSI state-space inverter
- DP: add averaged-VSI state-space inverter component (#545), adding a cxx example (#546) and a cross-domain validation notebook (#547)
- Add DP::Ph3::AvVoltSourceInverterStateSpace (#579), adding negative-sequence current control (#592) and unbalanced single-line-to-ground validation (#586)
- Feature: New exciter models (#402)
- Feature: New PSS (#486)
- Feature: steam turbine governor and base governor and turbine (#491), and hydro turbine governor (#492)
- feat: integrate TurbineGovernorType1 with SynchronGenerator and add Python bindings (#490)
- Feature: sg controllers validation and tests notebook (#494)
- SP SynchronGenerator: add reactive-power limits Qmin and Qmax (#535)
- DP_Ph3_PiLine, EMT_Ph1_PiLine and Ph3 diakoptics functionality (#436)
- Add DP_Ph1_Shunt and EMT_Ph3_Shunt components (#550)
- DP_Ph3: add CurrentSource (#538), taking an independent per-phase reference in VoltageSource and CurrentSource instead of a single value rotated by a hardcoded 120 degrees
- Add DP::Ph3::Switch three-phase matrix switch (#583)
- Feat: Add Scalar EMT DC Components (#630)
- Decoupling ITM (#423)
- feat: sparse-Jacobian powerflow solver with KLU refactorization reuse for large grids (#514)
- PFSolver: enforce generator Q-limits via a PV to PQ outer loop (#536)
- feat: configurable PFSolver base-power fallback and working max-iterations setter (#552)
- Diakoptics development: KLU-based subnet solving with localized system matrices (#513)
- feat: opt-in filter for out-of-service equipment and isolated buses in matpower reader (#553)
- Feat: Add option to map synchronous machine to ideal voltage source in matpower (#601)
- Implement functions for mapDisconnector and mapBreaker (#556)
- Add IEEE 9-bus 4th-order SG examples in SP, DP and EMT (#575)
- Build and publish Windows wheels (#633)
- Feature: auto-generate .pyi stub files for dpsimpy via pybind11-stubgen (#495)
- docs: add CONTRIBUTING.md and a notebook output compliance check (#509)
- Add attribute usage guidelines (#503)
- docs: document MNA solver and component initialization (#505)
- Document the DP::Ph1 (#549) and DP::Ph3 (#591) averaged-VSI state-space inverters
- Add code owners for state-space source files (#481)
- ci: add multi-stage LLM PR reviewer (#594), tiering the models across finders and escalating verify gates (#595)
- ci: enforce conventional commits for commit messages (#607), checked by CI and by a commit-msg hook
- add: run binder container test by running examples/villas/dpsim-file.py (#645)

### Changed

- Refactor EMT 3-phase state-space inverter to variable SSN (#477)
- PFSolver: zone-based base voltage verification, configurable tolerances, and matpower Q-limit fixes (#543)
- Refactor: remove duplicate initialize(mFrequencies) calls and migrate SVC to CompositePowerComp (#511)
- refactor: shared elementwise complex-product helper (#584)
- SystemTopology: derive mComponentsAtNode instead of hand-maintaining it (#638)
- Migrate Python project config to pyproject.toml (#485)
- chore: strip saved outputs and backfill cell ids across all notebooks (#605)
- docs: restructure the documentation and close the coverage gaps (#619)
- docs: expand task scheduling page (#506)
- update: improve .gitignore coverage (#483)
- Changed where the container workflow runs to a private runner (#465)
- Fix: Bumped CI to node 24 actions (#484)
- ci: cancel superseded workflow runs via concurrency groups (#534), so a re-push cancels the in-progress run
- ci: patch and dev suffix for Test PyPI publishes (#565)
- Analyze fork PRs with SonarCloud without exposing secrets (#593)
- ci: run every workflow from a single CI entry point (#661), and run fork pull requests through the same pipeline as the rest (#674)
- ci: cache the container images in the registry instead of the Actions cache (#672)
- ci: resolve pyproject extras with uv instead of pip-tools (#650), and resolve Python dependencies with uv in the Rocky container (#651) and in the remaining images and scripts (#670)
- build(python): drop 3.9 and build wheels for 3.14 (#681), documenting the supported version matrix and the Windows wheel limits on the install page
- CODEOWNERS: update for the next release (#637), applying the removal and ownership requests collected from the maintainers and repointing the paths that moved
- fix(ci): drop the build-time Python dependency layer from the manylinux image (#648)
- test(examples): skip WSCC 9-bus switch notebook while its reference URL is dead (#665)
- ci(llm-review): stop repeating findings on every push (#676)
- Scope cpp:S5184 out of the pybind bindings in the Sonar configuration (#678), since it targets unnamed RAII temporaries and the py::class_ registration idiom holds no resource
- Bump picomatch and postcss in /docs/hugo (#464, #471, #599)

### Removed

- Remove outdated roadmap (#463)

### Fixed

- Fix EMT 3Ph Switch behaviour for Matrix Recomputation (#419)
- Fix angle conversion in Math::polarDeg (#473)
- Fix pybind on windows builds (#470)
- Fix Windows build with std::filesystem (#480)
- fix: TurbineGovernorType1 initialization attributes (#493)
- fix: virtual node ordering for composite components (#496)
- Bugfix for 2 minor issues in DiakopticsSolver (#507), and fixing collectVirtualNodes to recurse into subcomponent virtual nodes (#523)
- fix: PF Newton-Raphson globalization and matpower reader and notebook CI fixes (#512)
- fix: correct N0.V filter in DP_CIGRE_MV_withDG_withLoadStep notebook (#510)
- fix: SP::Ph1::Capacitor produces NaN admittance for zero capacitance (#519)
- fix: guard PF convergence check against non-finite mismatch (#520)
- fix: refresh matrix-node-index cache after terminal swap in SP Transformer PF (#521)
- SP power-flow: remove dead isinf guards and unused capacitor fields (#525), rerouting them through Math::isFinite, which survives the -Ofast build, and through the logger instead of std::cout
- SP transformer: fix snubber NaN from missing or invalid rated power (#532), sizing the matpower transformer rated power from the branch rateA
- fix: add missing Shunt branch to PFSolver::determineNodeBaseVoltages() (#551)
- fix: exclude MNA fallback shunt from standalone PiLine powerflow stamp (#555)
- feat: restore EnergyConsumer SSH-derived P/Q reading in CIM reader (#554)
- fix: CIM reader PQ-generator mapping and slack voltage unit bug (#558)
- fix: ExternalNetworkInjection voltage target unit disambiguation (#562)
- DP: fix double carrier-shift in MixedVTypeVariableSSNComp init (#567)
- fix(models): linearize the SSN grid-forming inverter analytically (#642)
- fix: re-pivot fully when no variable matrix entries are declared (#644)
- SystemTopology::removeNode: remove connected components too (#621), and removeComponent: drop the component from mComponentsAtNode too (#634)
- pybind: restore SystemTopology.add as a deprecated alias (#622)
- fix(cmake): stop passing GCC optimization flags to MSVC (#641)
- fix(build): pin the wheel build to the target interpreter for pybind11 (#649)
- fix(docker): build libcimpp from source on ARM64/macOS (#610)
- fix: missing links related to suitesparse (#482)
- fix(test): unbreak notebook collection and pre-commit on master (#652)
- fix(ci): publish sogno/dpsim:dev-vscode and use it for the devcontainer (#659)
- ci: LLM reviewer verify stage keeps missing-SPDX findings (#596), and handles oversized diffs and unresolvable inline lines (#597)
- ci: fix Sonar scan metadata resolution for fork PRs (#598)
- fix(ci): let the approved fork gate check out fork pull requests (#671)
- fix(ci): stop building the images against the RWTH GitLab (#673)
- fix(ci): install libxil in the manylinux image from GitHub (#675)
- Zero-initialize the turbine governor parameters (#678)
- Move instead of forward the by-value attribute dependencies (#678)
- Drop the spaces around the LABEL assignments in the shmem image (#678), which collapsed all four labels into a single one carrying the rest of the block as its value
- restore github hosted runners in workflows (#682)

## [v1.2.1] - 2025-12-10

Note: this version only includes a fix to the publishing workflow. See v1.2.0 for relevant changes since the last v1.1.1

### Fixed

- TestPyPI and PyPI were part of the same job, solved a mixup/reuse of attestations (#459)

## [v1.2.0] - 2025-12-10

### Added

- Added this CHANGELOG.md file to track changes in the project.
- Adapt DPsim to deploy it in Mybinder (#323), moving Dockerfile to new .binder folder and adding job for building the binder Dockerfile.
- Add Code Coverage Report (#395), adding codecove badge and updating yaml file for code coverage.
- Add CODEOWNERS (#290)
- Add pre-commit (#352, #361, #363, #377, #382, #383, #385, #386) to harmonize code
- feat(cmake): Add install target (#381)
- New queueless VILLAS interface / improve real-time performance (#316), creating DCGenerator in VoltageSource only if special setParameters is used and not always when the frequency is 0 (avoiding regressions), and adding DP_PH1_ProfileVoltageSource used for the villas / fpga interface.
- Reuse conductance stamp code (#306), adding functions in MNAStampUtils for stamping value as a scalar matrix.
- Supress a warning treated as an error in MSVC (#280)
- update contact details (#254), updating contact details.
- Add support for CMake `find_package` and rename "KLU" dependency to name of actual library "SuiteSparse" (#380)
- Addition of EMT::Ph1::Switch component (#312), adding test notebook for VS_SW_RL1 circuit in EMT, DP and SP and fixing initialization.
- Linear SSN (#175), making EMT::Ph3 CurrentSource and SSN Full_Serial_RLC usable in pybind with implementation and declaration adjustments for EMT Ph3 CurrentSource.
- Implement decoupling line emt ph3 (#422), improving nine bus decoupling example with 3ph decoupling line and adding test for decoupling line Ph3 to Line.ipynb example.
- VILLASfpga cosimulation development (#325), adding n-eiling as codeowner of /dpsim-villas and fixing MNASolver to start loggers for vectors.
- Villas Interface: Improve FpgaExample (#299), allowing different topologies to be tested and updating VILLAS_VERSION in Dockerfile.
- Nix packaging (#357), adding GitHub CI workflow for building DPsim with Nix and Nix packaging.
- chore: update docs on real-time and VILLASnode interfaces (#335)
- Add dpsimpyvillas module to the CMake targets and include it in the Python wheel, using the correct Python install dependencies (#449)

### Changed

- Change python to python3 to run the building for pypi (#272)
- Disable code coverage in linux-fedora-examples on workflow (#412)
- editorconfig: set C++ indentation size to two (#295), setting it twice.
- Move pytest.ini to pyproject.toml (#348)
- Reduce attributes number (#388), declaring nominal/base voltages as Real instead of Attribute<Real>.
- Replace CentOS badge with RockyLinux badge (#282)
- villas: Use Python dictionaries for defining VILLASconfig files (#343)
- Change default linear solver to KLU (#250), fixing logging statements by using macros and removing explicit choice of SparseLU from cxx examples.
- Update WSCC_9bus_mult examples and implement splitting examples in DP and EMT (#322), removing SP_Ph1_CurrentSource component and fixing path to CIM files in decoupling line diakoptics example.
- Upgrade Fedora dockerfiles to v42 (#387), fixing KLU Adapter varying entry index check and adding libre to Dockerfile to fix one of the real-time errors using villas.
- Add new action using the build wrapper from sonar directly (#296)
- Allow builds without nlohmann/json library (#362)
- feat(ci): Use trusted / OIDC publishing to PyPi.org (#375), only attempting upload to PyPi for pushes to master or tags.
- Fixes to documentation and README, adding extra details about installation methods, contribution guidelines and how to try DPsim (#457).
- Update documentation and packaging metadata contact information, add CONTRIBUTORS file and GitHub Discussions link, and improve contributor/community guidance (#429, #413).
- Extend the changelog to include entries for older release versions and previously unreleased changes (#430).
- Use the DPsim manylinux container for building the Python package and skip RC versions when publishing to PyPI.org, while still publishing RC versions to TestPyPI (#451).

### Removed

- Remove commented out code (#390)
- Remove further RWTH gitlab dependencies (#261)
- Remove Python versions 3.6 and 3.7 for PyPi upload (#229)
- Disable test actions for PRs (#237)
- chore(cmake): Remove deprecated options (#342)
- Remove outdated and broken GitLab CI pipeline (#347)

### Fixed

- 5th Order Synchronous Generator (#230), fixing some minor details and adding SG5Order to CIM Reader in DP and EMT domain.
- Adding correct logger usage (#411)
- dpsim-villas: fix hardcoded paths (#327)
- Enable compilation in gcc 14 and clang 18 (code fixes) (#294)
- feat(cmake): Allow disable LTO builds (#341), adding WITH_MARCH_NATIVE option to enable native host arch builds.
- Fix consistency of calculations at first time step (#210), using Ph3 RMS quantities for initialSingleVoltage in EMT domain and adding macro DOUBLE_EPSILON.
- Fix docu issues (#310), substituting duplicate ODESolver with DAESolver in dpsim_classes_simulation.svg.
- Fix docu pipeline (#333)
- Fix initialization rxload (#241), fixing current initialization of EMT RXLoad.
- Fix module dpsimvillas (#401), fixing ambiguous use of VoltageNorton->SetParameters and fixing examples for realtime and CIM usage.
- Fix power flow initialisation of synchronous generators (#238), fixing pybind double definition of components_at_node and enforcing domain argument for initWithPowerflow.
- Fix rocky workflow due to changes in the profiling triggering (#305)
- Fix some capitalization (#351)
- Fix sonar scanner errors due to Java deprecated version (#265)
- Fix submodule initialization for CIMpp in Dockerfiles (#331), fixing it twice.
- Fix the profiling workflow (#399)
- Fix VILLASnode in Dockerfiles (#292), updating VILLASnode version in Dockerfile.manylinux and in other Dockerfiles.
- Fix workflow: option for parallel in make and error in publish to PyPI (#303)
- fix(cmake): Show feature summary even if not Git info is available (#379)
- fix: MNASolverPlugins examples (#346), fixing include paths of MNASolverPlugin examples and wrapping extern c in preprocessor condition.
- Fixes to the doxygen documentation INPUT line (#264)
- Fixing inconsistent switch attribute names in Base::Ph1::Switch and Base::Ph3::Switch (#418)
- Fixup whitespaces (#350)
- Hotfix: missing simulation stop (#247), adding missing stop function in the Simulation using pybind.
- pybind: Errors when compiling with clang (#334), fixing rocky container and updating villas version, fixing fedora container error in mosquito install and updating villas version.
- Reuse code for MNA matrix stamp operations (#297), logging stamping in MNAStampUtils and reusing admittance stamping logic for SP R,L,C components.
- Reuse stamp code for EMT synchron. generators (#315), adding matrix stamp functions with optimized logic in MNAStampUtils and fixing stamping of full matrix.
- Revision of power flow solver (#284), fixing PFSolver and minimizing file changes.
- Trigger actions for PRs and pin the version of libcimpp (#329), pinning libcimpp version in containers and cmake and triggering relevant workflows on pull_request event.
- Update actions in workflow and improve parallelisation and cache handling (#298), changing the profiling execution to manually triggered and enabling parallelisation with reorganized cache.
- Bump braces from 3.0.2 to 3.0.3 in /docs/hugo (#304)
- Bump postcss from 8.4.20 to 8.4.31 in /docs/hugo (#281)
- Bump version of black & black jupyter to 25.11.0 (#421)
- Bump yaml from 2.1.3 to 2.2.2 in /docs/hugo (#215)
- fix clang compilation and FPGA integration (#293), fixing clang compiler errors and adding clang compilation to CI.
- Fix CMake typos and style (#376)
- Fix GitHub actions workflow for publishing to PyPi (#355), building with newer CMake versions and simplifying triggers for workflows.
- Fix realtime datalogger (#400), adding example notebook for tests and pybind bindings.
- fix std::max behaviour in simulation.cpp for windows build using CMake (#408), fixing issue where windows.h breaks standard C++ behaviour of std::max.
- Fix the villas examples workflow (#319), adding supplementary cache and testing cache deletion.
- Fix Windows and Rocky/Clang builds (#360), fixing Windows builds with newer CMake versions.
- fix(ci): Attempt to fix PyPi package publish (#353)
- fix(ci): Fix tag of pypa/gh-action-pypi-publish action (#354), fixing the tag twice.
- fix(cmake): Fix code-style (#356)
- fix(deps): Add support for newer Spdlog versions (#340), adding missing #pragma once.
- fix(docs): Fix typo and wrong diff (#337)
- fix(examples): Only build Fpga9BusHil example if build with CIMpp (#338), applying it twice.
- fix(style): Remove duplicated typedef (#339)
- python: Harmonize comment style (#349)
- chore(deps): Bump pypa/gh-action-pypi-publish from 1.12.4 to 1.13.0 in /.github/workflows (#405)
- Update villas version (#245), updating python command to python3 in test-villas-examples actions and removing libwebsockets installation from source.
- Use clang-format to format the whole codebase (#278), fixing missing includes and moving development scripts from configs/ to scripts./
- chore (deps): update to CIMpp rpm/deb install + bump VILLAS (#404), updating cmake fetch and finding of libcimpp and fixing dependent files and CIMReader.
- fix: Cleanup code-style of setup.py (#336)
- Reactivate VILLAS support in the MyBinder Python packaging workflow and fix related compilation issues when using dpsimpy with VILLAS-enabled containers (#450, #434, resolves #445).
- Create a symlink in the manylinux image to the install path of the OpenDSS C library to fix Python packaging and runtime loading issues (#448).
- Link DPsim to VILLAS libraries via CMake to fix missing linkage in VILLAS-enabled builds and examples (#447).
- Make the Fedora release workflow wait for the dpsim-dev container build to avoid race conditions in the release pipeline (#446).
- Security fix for the profiling workflow, avoiding passing commands as inputs to scripts in the profiling job (#455).
- Security fix for the VILLAS workflow, preventing commands from being passed as raw inputs into the shell (#454).
- Adapt the Sonar workflow to avoid deprecated secret checks so that scans still run for pull requests from forks (#456).
- Fix the Sonar workflow by bumping the action to the supported v6 version and adjusting configuration (#439).

## [v1.1.1] - 2023-07-13

### Added

- Add latest publications to website (#161)
- Introduce a common subclass for composite components (#177), adding comments for empty method implementations and renaming systemMatrixStamp methods for sparse matrices.
- Build and push RockyLinux image (#148)
- Update and expand documentation (#159), adding documentation on MNASimPowerComp and subcomponent handling.
- Introduce common base class MNASimPowerComp (#178), fixing merge errors and addressing SonarCloud issues.

### Changed

- Unify subcomponent handling and call MNA methods through the parent (#141)
- Move documentation to main repo (#154), adding hugo website files and moving content from dpsim-docs repo.
- Refactoring of direct linear solvers in MNA solver (#171), addressing further pull request comments and adding fetching of minimized suitesparse.

### Fixed

- Configurable linear solver (#199), addressing further pull request comments and processing default configuration options.
- Fix EMT_Ph1_Inductor initialization (#134), using Modelica results for validation and adding current validation in example VS_RL1.
- fix hugo website deployment (#158)
- Fix missing 'this' in logger macros (#217)
- Fix network frequency modulations and vsi snubber comps (#218), fixing some logging issues and trafo instantiation in dp and sp vsi for proper snubber comps.
- Fix various issues related to attributes (#140), addressing SonarCloud code smells and moving method implementations into source files.
- Profiling based optimisation (#194), addressing pull request comments and applying switched component stamps.
- Resolve FIXME comments (#142), addressing SonarCloud issues and renaming mMechPower attribute.
- Cleanup of CMake and Docker files (#168), adding missing libcimpp dependency for cimviz and comment explaining CMAKE_POLICY_DEFAULT_CMP0077.
- Deploy docs to FEIN website (#160), fixing repo url and subdir.
- fix and update documentation (#165), updating Contribution Guidelines and Roadmap.
- Fix implementation of exciter, new turbine governor model and refactor VBR models (#120), applying minor fixes and replacing validation notebook of reduced order SG models.
- Reduced-order syngen models (#213), removing unused variable in MNASolverDirect and unused methods/variables of MNASyncGenInterface.

## [v1.1.0] - 2022-12-09

### Added

- Add a basic VS Code devcontainer configuration
- add cppcheck for use in github actions, updating Dockerfile and install shell scripts and adding docker-compose for dpsim-mqtt examples.
- Add cps files to dpsim repo
- add deployment to gitlab pages
- add missing file headers, updating copyright year to 2021.
- Add new working mqtt example
- add notebook checking powerflow of ieee lv, adding notebook checking powerflow for cigre mv.
- add pillow dependencies to centos dockerfile
- add powerflow cim example
- add villas import example, using PQ attributes in SP load.
- add vsi model and examples
- allow vd, pv and pq comp at the same bus, setting bus type to vd, updating submodule grid-data and updating syngen model in sp1ph.
- avoid deadlock, adding log prefix and console logger for simulation and shmem interface.
- docker: add entry page to docker image, copying examples into image and starting Jupyter Lab by default.
- made all power components set parametersSet flag, adding setParameters flag and function with new constructor.
- python: add missing Compontents (closes #118)
- require only cmake 3.11 and make policies requiring a newer version optional
- shmem: update Python examples, allowing exporting of attributes with name & unit.
- sphinx: add sections in reference
- update cps, only declaring node templates in cpp for win and adding description type to setup.py.
- update steady state init and add logging

### Changed

- cmake dependent opt causes problem with cim
- cmake: use cmake_dependent_option() to avoid caching issues
- Create vnodes in pcomps instead of MNA solver and differentiate between 1ph and 3ph for the simNode assignment.
- flush spdlog after sim initialization, switching case for python log level and adding function to get current voltage.
- force gcc 11 in manylinux image
- initialize frequencies in SystemTopology
- Merge branch 'development' into 'master'
- Merge branch 'master' into multisampling, updating notebooks and gitignore.
- Merge branch 'patch-1' into 'master'
- Merge pull request #60 from JTS22/python-spdlog
- MNASolver: Increase maximum switch number to theoretical limit
- move libcps to models
- notebooks: clear results
- run autodoc on python submodules
- run page deploy on dedicated branch
- silence experimental fs visual studio warning
- started working on developer documentation
- Trigger PyPi-Workflow for tags (#151)
- update 9bus parallel notebook
- update cps, updating shmem distributed direct nb and shmem direct examples.
- update file headers and replace gpl with mpl in most files
- update inverter notebooks
- update shmem example of cigre mv
- update submodules path, updating dpsim results path in notebooks.
- rename Terminal, renaming TopologicalComponent and Node class.
- disable cim and render if deps not available, removing obsolete cim reader includes and working around github permission issue.
- update ci, restructuring sphinx doxygen docs and removing notebook related sphinx cmake.
- update cps, merging master and restructuring notebooks.
- update docs url
- update global scope of SynGenTrStab examples, adding Events category to examples and splitting examples in two categories: components & grids.
- update install script and new dockerfile, updating docker dev and using libxml instead of libexpat in cimpp.
- update shmem examples, using new cpp export methods in python interface and adding jupyter requirements file to docker fs.
- updated installation instructions, showing cmake invocation during setup.py and allowing setting CMake options via envvar.
- use correct minimum cmake version (3.13 instead of 3.12) because Policy CMP0076 requires 3.13, using different cache key for build with and without cuda and making CI build with CUDA support with CUDA dependencies in Dockerfile.dev.
- use updated dpsim-villas and villas-node versions, removing pipeline build caches and updating fedora and pybind versions.

### Deprecated

- moving old examples into villas-deprecated folder.

### Removed

- disable macos build
- harmonize namespace aliases of std::filesystem, removing useless printing of working directory in examples.

### Fixed

- add sst model and fix powerflow
- append .git to all module urls, fixing path to cim grid data and updating README.
- examples: fix notebook bin path, considering load "P" attribute in PF during simulation.
- fix brief class descriptions of reduced-order vbr models, fixing sp shift and adding grid voltage evaluations in validation notebooks, fixing cmd options after rebase.
- fix cim path for shmem example of cigre mv
- fix cmake policy warning
- fix dp ph3 voltagesource stamp
- Fix dpsim-villas version tag, using new villas version.
- fix GPLv3 logo (closes #125)
- fix load profile generation for different time steps in cigre shmem example, fixing cim files path and extending CLI options.
- fix minimal dockerfile
- fix powerflow with load profiles and use csvreader instead of loadprofilereader
- fix rxload initialization
- fix sourcevec writing and update villas-dataprocessing
- fixing mechanical torque as attribute after rebasing, adding logging of indices of varying matrix entries for reduced order VBR models in DP and EMT and adding smib reduced order load step examples.
- Merge branch 'debug-msp-example', updating cps and adding notebook with CS RL1 circuit.
- merge master, updating cps and using cmake_dependent_option() to avoid caching issues.
- MNASolverFactory.h: Only use Plugin Solver if activated, adding *.so to .gitignore and making MNASolverPlugin optional and deactivated on windows.
- python: fix invalid return value for Node::setInitialVoltage()
- spdlog macros, fix simNode assignment of vnodes, set links to master branch
- switch shared library off to fix dpsim_python, updating editorconfig and adding numpy cmake option again.
- update cps, ignoring attributes if index is larger than length and updating 9bus ctrl shmem example.
- update cps, fixing error in WSCC 9bus notebook and fixing feature notebooks.
- update readme, fixing shm python example and adding removed FindSphinx cmake file.
- update tlm comparison notebook, adding correct phase shift to transmission line and adding emt decoupling wave line model.
- use correct attribute dependency, making the phase changeable through mVoltageRef and using member variables for attribute access.
- use image for devcontainer, setting manual trigger for container workflow and fixing devcontainer.json.
- use initialization list for mZigZag, adding comments on voltage source behaviour and removing constructor for removing the SignalGenerator.
- add comments, adding another export to Simulation::sync and increasing timeout to allow dpsim to catch all mqtt messages.
- add link to build notebooks
- enable tests for pull requests, updating sonar settings and adding sonar cache and multithreading.
- examples: reduce sim time because of CI duration, fixing find pybind python in cmake.
- fix .gitlab-ci.yml to include correct path to rocky Dockerfile, fixing docker labels and removing duplicate needs statement.
- fix build badge
- fix doc gh-pages deployment
- fix emt ode syngen model, using systemtopolgy initFromPowerflow in examples and setting initFromNodes for MNA solver.
- fix file paths in notebook, updating notebook to use the new attribute system and exposing the entire attribute system to python.
- fix gitlab ci using old docker syntax (#147)
- Fix PyPi-Upload CI workflow (#149), finding Python3 instead of Python and updating container workflow actions.
- fix set pattern in scheduler, setting log pattern in scheduler and merging master.
- fix submodule build: throwing only warning when git version unavailable
- include Magma in Dockerfile, cleaning up gitlab-ci.yml, fixing GPUMagma inverse permutation bug and adding Magma based MNASolver.
- keep governor and exciter params separate from machine params, adding governor and exciter test examples for 9th order syngens and fixing exciter of dcim syngen.
- merge 1ph-refactoring, updating cps and fixing cpp 9 bus nb.
- Merge branch 'test-villas' into 'master', updating repo link in build docs and shmem example nb.
- Merge pull request #39 from stv0g/ci-minimal-build
- Merge pull request #46 from stv0g/ci-minimal-build
- minor tweaks to gitlab-ci
- move imports into cpp files, removing binary dir include for dpsimpy and clarifying Cpp version change.
- output dpsimpyvillas into top-level build directory, exporting complex attributes instead of matrices
- pybind: remove superfluous includes, removing unused VILLAS_VERSION variable from docker and allowing building examples without CIM support.
- re-enable notebook test, re-enabling example and adding missing test examples.
- reactivate continue_on_error parameter, adapting cigre mqtt example to run in pipeline and adding test for dpsim-mqtt-cigre example.
- refactor to store cli options as string
- remove cmake copy image, updating grid-data and examples with new grid data path.
- remove debug info from workflow, removing redundant jobs from gitlab and cleaning up dockerfiles.
- remove obsolete gitlab CI jobs, generating sphinx docs for new python interface.
- Revert :Fix compilation without libcimpp and minor tweaks to CMake code-style
- skip tests for old notebooks, adapting quickstart guide to dpsimpy and triggering actions on push to any branch.
- Use read-the-docs sphinx theme

## [v1.0.0] - 2019-04-15

### Changed

- Merge branch 'dae-solver-test' into 'development', adding documentation on real-time execution of DPsim (closes #108) and merging development into dae-solver-test.
- Merge branch 'gen-arkode' into 'development', updating Simulation.cpp and DataLogger.cpp.

### Fixed

- Continued to fix odeint example problem, adding example program for odeint based on DP_SynGen_dq_ThreePhFault of Arkode and first implementation of odeint solver class.
- Fixed initial value setting of odeint solver
- adaptations for DQ SynGen class split, merging branch 'development' into parallel and updating cps submodule.
- fixes for clang, adding parallel multimachine benchmark and fixing memleak in ODESolver.
- ifdef for sim_ode, updating .gitmodules and cps.
- include nbs in docs
- merge powerflow, applying minor logging fix in cigre mv powerflow test and writing config of data sent via villas.
- update cps, merging branch 'development' into powerflow-integration and updating dockerfile.dev.

## [v0.1.6] - 2018-11-04

## [v0.1.5] - 2018-11-03

### Added

- add missing _dpsim module to binary Python packages
- cmake: only integrate interface code if the platform supports it, adding more parameters to DPsim::RealTimeSimulation::run() and moving timer creation into {create,destroy}Timer().
- docker: enable jupyter widgets
- python: add missing invocation of loggers to python simulation loop, adding proper reference counting for logged Components/Nodes.
- python: add missing progressbar package as dependency, pausing and resuming simulation asynchronously and adding more async events.
- update CPS submodule, providing synchronization settings per interface and making start script executable.
- updated cmake files for cimpp, including cimpp submodule in make files and adding libcimpp as submodule.
- updating cps, updating freq and load step example and removing NZ publication example.
- updating emt example, merging branch 'development' into msv-pra and updating examples.

### Changed

- flush data logs on simulation end, updating notebooks and raising error when dpsim.Simulation.run() is used with an already running event loop.
- Install expat dependency and execute setup
- Merge branch 'redesign_cimreader' into 'development'
- merge updated circuit examples, updating examples.
- python: do not fail load _dpsim.Interface (closes #93), reverting "comment out interface python class".
- python: separate EventChannel in header/source
- updated copyright year in C++ example
- merge changes into development, updating .gitlab-ci.yml and deactivating windows build until we have a runner again.
- merging changes in mna solver, using new DataLogger and making MNASolver not rely on CIM::Reader.
- merging changes into development, merging branch 'refactor-move-cps' into 'development' and not crashing if there are no CIM tests.
- merging new commits from development and node-terminal update, updating CPowerSystems submodule and refactoring Add{Excitor,Goveronor} -> add{Excitor,Goveronor}.
- Update Build.rst
- updating libcps, merging attribute test and merging redesign-simulation into development.
- shmem: libvillas-ext has been obsoleted by libvillas, simplifying dpsim.Simulation.run() and fixing error messages in CPS::Python::Component:getattro().

### Fixed

- Adapted DAEsolver to new node/component structure and removed IDA dependency of Simulation file, making Residual function static and integrating DAE into Solver structure.
- docker: add Dockerfile for building a ready-to-run version of DPsim, fixing RPM dependencies and building RPM packages.
- docker: use fedora 29, fixing compilation without VILLASnode in python.
- fix DPsim logo in README.md
- fixing namespaces in simulation.cpp, updating CPS submodule for fixing linking errors and moving Python bindings for components to CPS.
- last buf fixes before merging, improving usage infos for command line args and adding missing header file.
- Merge branch 'development', changing shmem ids and increasing simulation time in wscc.
- python: fix logging of node voltages (closes #97)
- update CPS submodule, fixing attribute names in examples and python tests.
- fix windows build
- fixing CIM test yml, xfail for larger CIM example and removing TopologicalIsland from examples.
- Fixing parallel support of node indices and CIM, removing deprecated line load test and fixing network description without node objects.
- fixing segfault in mna solver when getting nodes, adding new examples and updating cps.
- Merge branch 'development', merging generator examples and updating cps.
- python: add some documentation to factory functions, fixing unit tests and changing NULL -> nullptr.
- tests: renamed python script in order to be selected by pytest, updating Python version of ShmemDistributedDirect test and refactoring examples for new lambda interface.

## [v0.1.3] - 2018-02-21

### Changed

- Update CMakeLists.txt
- refactor: DPsim::Components::Base -> DPsim::Component (closes #44), renaming "EMT_VoltageSourceNorton.{h, cpp}" to "EMT_VoltageSource_Norton.{h,cpp}" (see #43).

### Fixed

- fixed mistakes after merging, using SynGenSimulation for VBR simulation and integrating new DP VBR model with nodal analysis.
- Merge branch 'fix-transformer' into development merge, fixing CIM reader and example and applying minor changes in logging.
- examples: disable another failing CIM test, disabling broken IEEE-9-bus CIM test and refactoring mSeq -> mSequence.
- version bump, fixing url and image links in readme.

## [v0.1.1] - 2018-01-12

### Added

- added missing license and copyright headers (closes #23)
- TurbineGovernor: Added init function to initialize governor variables
- Update DP_ResVS_RXLine1.cpp, merging branch 'development' into 'shared-factory' and applying smaller cleanups.

### Changed

- do not use shared_ptrs for simulation Loggers
- Update .gitlab-ci.yml
- Update FaultSimulation.cpp, updating Simulation.cpp and merging branch 'development' into 'refactor-component-element-naming'.
- updated logdataline, fixing capitalization of filenames and excluding certain parts from Doxygen as they break Breathe.

### Removed

- refactor: rename namespace "DPsim::Component" -> "DPsim::Components", removing default constructor from components and naming base class files "Base_*.h".
- simulation: remove obsolete Generator test classes, simplifying expression and fixing real time simulation support in python.

### Fixed

- examples: fix hardcoded path (this is still ugly), installing libraries to correct location on Fedora and adding setup.py for installing / packaging with setuptools.
- fixed test builds
- Merge branch 'python-log' into 'development', fixing Python CI test and python ctor of Simulation class.

## [v0.1.0] - 2017-12-26

### Added

- changed newNode to node3, adding comments to header files of capacitor, ideal voltage source, inductor, inductor2 and voltSourceRes and adding documentation for ideal and real voltage source.
- created new components classes
- loglevel filter, updating study scripts and adding svg figures.
- Merge branch 'dev-vsa' into 'master', adding figures for synchronous machine and synchronous generator figure.
- Merge branch 'dev-vsa' into 'master', changing capacitor and inductor in simulation models and updating figure of inductor model.
- merge dev-mmi into master, documenting EMT DP comparison and first version of comparison between EMT and DP.
- moved Simulink models to other repo, adding folder DPsimReferenceExamples and deleting folder SimulinkExamples.
- Update LICENSE, adding license.
- VBR DP, merging branch 'development' and adding logo.
- Added omega_base to the equations of trapezoidal rule and implemented trapezoidal rule with current as state variable, implementing trapezoidal rule with flux as state for EMT and correcting mistake in equation of ifd.
- Created voltage behind reactance model - in construction, adding trapezoidal rule to DP synchronous generator and merging branch 'rt-exceptions' into development.

### Changed

- DP VBR working
- Merge branch 'development', merging master into dev-mmi merge and updating matlab scripts.
- Merge branch 'master' of git.rwth-aachen.de:PowerSystemSimulation/DPsim merge
- Update README.md
- VBR euler, adding matlab script to compare plecs with c++ results for synchronous generator and only compiling shmem/RT parts on Linux.
- VBR model - simulation of steady state
- adjust CMakeLists.txt, merging branch 'development' and merging branch 'cim-xml' into development.
- Merge branch 'dev-vsa' into 'master', updating LinearResistor.cpp and BaseComponent.h.

### Removed

- deleted vs project files, cmake works now in vs, adding vs folder to gitignore and deleting vs installation md file, updating build.rst.

### Fixed

- added GPLv3 headers to source files (closes #9), moving Source/Examples to Examples/ and treating unused variables as errors in Clang.
- Created Ideal Voltage Source EMT and fixed "virtual node", simplifying models with operational and fundamental parameters.
- fix compiler warnings (closes #15), creating simplified model of synchronous machine and making all generator models use SynchGenBase.
- implemented exciter and turbine to VBR DP model, fixing mistake in fault clearing function and adding Turbine Governor model.
- pass matrices via reference to silence compiler errors, adding Dockerfile and GitLab CI configuration and searching in standard location for Eigen.
