```{seo}
:description: Learn how to compile Duckietown project documentation with Jupyter Book, mix Markdown with reStructuredText, and optionally autogenerate API docs.
:keywords: Duckietown, documentation, Jupyter Book, Sphinx, eval‑rst, API docs, dts docs build
```
(sec:dt_way_build_docs)=
# Building code documentation

```{needget}
* [Book‑Writer manual](book-opmanual-docs:duckumentation-intro)
---
* Ability to build and preview a Jupyter Book for your project
```

Duckietown projects ship with a ready‑to‑use Jupyter Book skeleton under **`/docs/src`**. Refer to the [Book Writer Manual](book-opmanual-docs:duckumentation-intro) for details on the supported features and a syntax cheat sheet.


## Quick build command

From the project root:

```bash
dts docs build
```

`dts` launches a container, resolves dependencies, and writes the rendered HTML to **`/docs/html`**.


## Mixing reStructuredText inside Markdown

MyST supports the [`{eval-rst}` directive](myst-parser:syntax/directives/parsing), enabling adding raw rST blocks in a `.md` file — useful when requiring constructs that Markdown lacks.

````md
```{eval-rst}
.. note::
   A note written in reStructuredText.
.. include:: ./include-rst.rst
```
````

**Rendered result**
```{eval-rst}
.. note::
   A note written in reStructuredText.
.. include:: ./include-rst.rst
```
---

## Automatic API generation *(advanced)*

Sphinx can parse docstrings and create an API reference automatically.  

To enable this:

1. Add [`sphinx.ext.autodoc`](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html) and, if desired, `autodoc_typehints` to `docs/src/conf.py`.  

2. Create stub files (e.g., `reference/api.rst`) that call `.. automodule::` or `.. autofunction::`.  

3. Re‑run `dts docs build`.

See the *Developers* section of the [official Jupyter Book guide](https://jupyterbook.org/en/stable/advanced/developers.html) for detailed instructions.


<!--
(sec:dt_way_build_docs)=
# Build documentation

You can write a Jupyter Book to document your project. The book source code is located in `/docs/src` and you can refer to the [Book Writer Manual](duckumentation-intro) for details on the supported features and a syntax cheat sheet.

To build the book you can simply run from the root of your project:

    dts docs build
    
## Including reStructuredText in Markdown

To insert rST into Markdown, you can use the [eval-rst directive](myst-parser:syntax/directives/parsing):

````md
```{eval-rst}
.. note::

   A note written in reStructuredText.

.. include:: ./include-rst.rst
```
````

```{eval-rst}
.. note::

   A note written in reStructuredText.

.. include:: ./include-rst.rst
```

## Automatic API generation (advanced)

It is also possible to automatically generate the API documentation from docstrings written in your source code. You can refer
to the [official guide](https://jupyterbook.org/en/stable/advanced/developers.html) for how to do this.
-->