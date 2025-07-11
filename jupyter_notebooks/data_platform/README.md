---
title: "Analyze Your Robotics Data with Jupyter Notebooks"
blog_post_url: "https://foxglove.dev/blog/analyze-your-robotics-data-with-jupyter-notebooks"
short_description: "Load and analyze data in Jupyter Notebooks using Foxglove Data Management"
---
# Jupyter notebook examples with Foxglove Data Management

This is a juypter notebook sample showing how to load and analyze data using [Foxglove Data Management](https://foxglove.dev/product/data-management).

## Run in Colab or Binder

Launch the sample notebook directly in Colab or Binder.

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/foxglove/jupyter-data-platform/blob/main/FoxgloveDataPlatform.ipynb)

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/foxglove/jupyter-data-platform/HEAD?labpath=FoxgloveDataPlatform.ipynb)

## Run in Docker

Alternatively you can launch the notebook locally via docker:

```
docker build . -f NotebookDockerfile -t jupyter-data-platform
docker run -it --rm -p 8888:8888 jupyter-data-platform
```

And then connect to the notebook via http://localhost:8888/lab/tree/FoxgloveDataPlatform.ipynb

## Run locally with pipenv

To run the notebook directly use following commands:

```bash
pip install pipenv
pipenv install
pipenv run jupyter-lab
```

## Stay in touch

Join our [Discord](https://foxglove.dev/community) to ask questions, share feedback, and stay up to date on what our team is working on.
