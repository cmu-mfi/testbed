This documentation is written in [Markdown][url01].

_Markdown_ files end in **.md**. You need a program to translate _Markdown_ code into **.html**. The Makefile and build scripts are written to execute the [Sphinx][url02] compiler, which is capable of translating **.md** files into a variety of target formats. It is also capable of extracting documentation from the source code of certain programming languages, and formatting that into
target representation.

## Installing _Sphinx_

_Sphinx_ installation documentation is available [here][url03].

Below are TLDR steps to build html pages locally.

**Install Dependencies**
```
git clone https://github.com/cmu-mfi/testbed.git
cd ./testbed
python -m venv ./.venv

# Linux/macOS Terminal
source .venv/bin/activate

# Windows CMD
call .venv\Scripts\activate.bat

pip install -U sphinx sphinx_rtd_theme myst_parser sphinx_copybutton sphinx_design
```

**Build doc**
```
cd testbed

# Linux/macOS Terminal
source .venv/bin/activate

# Windows CMD
call .venv\Scripts\activate.bat

cd doc
make html
```

The HTML pages are in `doc/_build/html`.  


[url01]: https://www.markdownguide.org/ "Markdown Homepage"
[url02]: https://www.sphinx-doc.org/ "Sphinx Homepage"
[url03]: https://www.sphinx-doc.org/en/master/usage/installation.html "Sphinx Installation Instructions"

