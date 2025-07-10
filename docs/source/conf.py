# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Embedded Systems Lab'
copyright = '2025, Universität Siegen'
author = 'Utkarsh Raj'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.mathjax",
    "myst_parser",
    "breathe",
    "sphinx.ext.todo",
]
myst_url_schemes = [
    "http",
    "https",
]  # Ensures that standard web protocols are recognized

myst_enable_extensions = [
    "dollarmath",  # Enables dollar symbol for inline math
    "amsmath",  # Allows more complex math formatting
]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# Breathe configuration
breathe_projects = {
    "Lab1": "../docs/xml"
}
breathe_default_project = "Lab1"

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static/unisiegen.png']
