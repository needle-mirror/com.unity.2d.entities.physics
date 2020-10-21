# Changelog
All notable changes to this package will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [0.1.0-preview.3] - 2020-10-21
### Added
* Runtime API
    * Added the following new types:
    	* `SimulationType` (enum)
    	* `ISimulation` (interface)
    	* `DefaultSimulation` (ISimulation)
    	* `NoSimulation` (ISimulation)

    * Added the following members:
		* `PhysicsSettings.SimulationType` (SimulationType)
		* `StepPhysicsWorldSystem.Simulation` (ISimulation)
		* `StepPhysicsWorldSystem.FinalSimulationJobHandle` (JobHandle)

* Authoring / Conversion API
		* `PhysicsSettingsAuthoring.SimulationType` (SimulationType)

