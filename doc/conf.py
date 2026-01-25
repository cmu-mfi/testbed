# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# project = 'MFI Hazelbots Testbed'
import time

project = 'Mezzanine Lab'
author = 'Carnegie Mellon University, Manufacturing Futures Institute'
copyright = '{}, {}'.format(time.strftime('%Y'), author)

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'myst_parser',
    'sphinx_copybutton',
    'sphinx_design',
    # 'sphinxcontrib.bibtex'
]

myst_enable_extensions = [
    'strikethrough'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
# html_logo = 'files/hbl_logo.png'
html_favicon = 'files/white-logo.ico'
html_static_path = ['_static']

html_css_files = [
    'custom.css',
    'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.1/css/all.min.css',
]

# -- top-right github link configuration -------------------------------------

html_context = {
    "display_github": True, # Integrate GitHub
    "github_user": "cmu-mfi", # Username
    "github_repo": "testbed", # Repo name
    "github_version": "master", # Version
    "conf_py_path": "/doc/", # Path in the checkout to the docs root
}

