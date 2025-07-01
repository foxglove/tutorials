import os
import re
import yaml
from jinja2 import Environment, FileSystemLoader

"""
A script to generate the README.md file for the project.
Execute it from the top-level directory.
"""

# Root directory of the project (current working directory when script is run)
ROOT_DIR = os.getcwd()
TEMPLATE_PATH = os.path.join(".utils", "README_template.md.j2")

CATEGORY_NAMES = {
    "datasets": "Datasets",
    "foxglove_sdk": "Foxglove SDK",
    "integrations": "Integrations",
    "jupyter_notebooks": "Jupyter Notebooks"
}

# Jinja2 setup
env = Environment(loader=FileSystemLoader(ROOT_DIR))
template = env.get_template(TEMPLATE_PATH)

def extract_metadata(readme_path):
    with open(readme_path, "r") as f:
        content = f.read()

    match = re.match(r"---\n(.*?)\n---", content, re.DOTALL)
    if not match:
        print("WARN: No YAML metadata found in", readme_path)
        return None

    try:
        metadata = yaml.safe_load(match.group(1))
    except yaml.YAMLError as e:
        print("Error parsing YAML metadata in", readme_path, ":", e)
        raise e

    return metadata

def scan_tutorials():
    tutorials = []
    for root, dirs, files in os.walk(ROOT_DIR):
        if "README.md" in files and root != ROOT_DIR:
            readme_path = os.path.join(root, "README.md")
            rel_path = os.path.relpath(readme_path, ROOT_DIR)
            metadata = extract_metadata(readme_path)
            if metadata:
                tutorials.append({
                    "title": metadata.get("title", "Untitled"),
                    "path": os.path.dirname(rel_path),
                    "short_description": metadata.get("short_description", ""),
                    "blog_post_url": metadata.get("blog_post_url", ""),
                    "video_url": metadata.get("video_url", ""),
                    "visualize_url": metadata.get("visualize_url", ""),
                })
    return sorted(tutorials, key=lambda t: t["path"])

def group_tutorials_by_category(tutorials):
    """Group tutorials by top-level directory (category), with special handling for integrations"""
    categories = {}

    for tutorial in tutorials:
        # Get the path parts
        path_parts = tutorial["path"].split("/")
        category = path_parts[0] if path_parts else "other"

        # Special handling for integrations directory with ROS1/ROS2 or other subdirectories
        if category == "integrations" and len(path_parts) >= 2:
            ros_version = path_parts[1]
            display_name = f"Integrations - {ros_version.upper()}"
        else:
            display_name = CATEGORY_NAMES.get(category, category.replace('_', ' ').title())

        if display_name not in categories:
            categories[display_name] = []
        categories[display_name].append(tutorial)

    # Sort categories and tutorials within each category
    sorted_categories = {}
    for category in sorted(categories.keys()):
        sorted_categories[category] = sorted(categories[category], key=lambda t: t["path"])

    return sorted_categories

def main():
    tutorials = scan_tutorials()
    categories = group_tutorials_by_category(tutorials)
    output = template.render(categories=categories)

    with open(os.path.join(ROOT_DIR, "README.md"), "w") as f:
        f.write(output)
    print("README.md generated with", len(tutorials), "tutorials in", len(categories), "categories.")

if __name__ == "__main__":
    main()
