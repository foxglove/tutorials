import os
import re
import yaml
from jinja2 import Environment, FileSystemLoader

"""
A script to generate the README.md file for the project.
Execute it from the top-leve directory.
"""

# Root directory of the project (current working directory when script is run)
ROOT_DIR = os.getcwd()
TEMPLATE_PATH = os.path.join(".utils", "README_template.md.j2")

# Category display names
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
        return None

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
                })
    return sorted(tutorials, key=lambda t: t["path"])

def group_tutorials_by_category(tutorials):
    """Group tutorials by top-level directory (category)"""
    categories = {}

    for tutorial in tutorials:
        # Get the top-level directory (category)
        path_parts = tutorial["path"].split("/")
        category = path_parts[0] if path_parts else "other"

        # Use the display name from constants, fallback to formatted name
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
