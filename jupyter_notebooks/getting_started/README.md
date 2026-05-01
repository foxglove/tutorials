---
title: "Get Started with the Foxglove Notebook Integration"
short_description: "Embed the Foxglove viewer in a Jupyter / Colab notebook and drive it with programmatic layouts"
# colab_url: "TODO_COLAB_URL"  # Uncomment and fill in once the Colab share link is available
---
# Foxglove Notebook Integration — Getting Started

A minimal, runnable companion to the [Foxglove notebook integration docs](https://docs.foxglove.dev/docs/notebook).

This notebook walks through:

- Installing `foxglove-sdk[notebook]`
- Creating a notebook buffer with `foxglove.init_notebook_buffer()`
- Logging messages and rendering them with the embedded Foxglove viewer
- Building layouts programmatically with the [`foxglove.layouts`](https://docs.foxglove.dev/docs/notebook/layouts) API, including nested `SplitContainer` and `TabContainer` examples driving real data

## Run in Colab

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](TODO_COLAB_URL)

> The Colab share URL will be filled in once the notebook is published. In the meantime you can run it locally — see below.

## Run locally

```bash
pip install "foxglove-sdk[notebook]" jupyterlab
jupyter lab GettingStarted.ipynb
```

The notebook works in JupyterLab, classic Jupyter, VS Code, and Google Colab — anywhere [`anywidget`](https://anywidget.dev/) is supported.

## Stay in touch

Join our [Discord](https://foxglove.dev/community) to ask questions, share feedback, and stay up to date on what our team is working on.
