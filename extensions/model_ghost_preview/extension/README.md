# Model Ghost Preview

Preview a topic's 3D model at the hovered playback time while the solid model stays at the current time.

The panel subscribes to the full message range of a pose topic, so the ghost still moves while playback is paused. The default model is a procedural haul truck. A glTF/GLB URL can be used instead.

## Develop

```sh
npm install
npm test
npm run build
npm run dev:harness
```

## Install

Install into Foxglove desktop:

```sh
npm run local-install
```

Or package a `.foxe` and drag it into Foxglove:

```sh
npm run package
```
