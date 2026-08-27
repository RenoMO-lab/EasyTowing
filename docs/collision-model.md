# Collision and Clearance Model

## Purpose

Provide a deterministic 2D clearance engine for rigid-planar steering designs.

## Envelope types

- `CircleEnvelope`: pivot keepouts, wheel centers with tire radius, round obstructions
- `CapsuleEnvelope`: rods, beams, and other swept line members with diameter
- `PolygonEnvelope`: chassis outlines, structural keepouts, and irregular packaging zones

## Clearance semantics

- Positive clearance means the envelopes are separated.
- Zero clearance means the envelopes are touching.
- Negative clearance means the envelopes overlap or violate the required margin.

## Required margin

Each collision item may carry an additional required clearance margin.
The analyzer reports:

- raw geometric clearance;
- required margin;
- adjusted clearance after margin;
- overlap detection.

## Current scope

The first pass supports the shapes needed for planar trailer steering packaging checks:

- circle-circle
- capsule-capsule
- circle-capsule
- polygon combinations

## Planned use

- tie rod vs pivot keepout
- tie rod vs axle beam
- steering arm vs axle
- wheel vs chassis
- wheel vs linkage
- drawbar vs frame
- bell crank vs surrounding structure

