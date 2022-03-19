# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# -- Project information -----------------------------------------------------

project = 'Vehicle Dynamics Control'
copyright = '2022, Alexander Wischnewski'
author = 'Alexander Wischnewski'

# The full version, including alpha/beta/rc tags
release = '0.0.1'

html_theme = 'sphinx_rtd_theme'
