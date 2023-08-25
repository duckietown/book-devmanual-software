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