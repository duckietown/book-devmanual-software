```{seo}
:description: Guidelines for writing high‑quality inline documentation in Duckietown ROS nodes and Python libraries, plus tips on using Sphinx to generate web‑ready docs.
:keywords: Duckietown, documentation, Sphinx, ROS nodes, Python, docstrings, autodoc, style guide
```

(sec:dt_way_code_docs)=
# Writing code documentation

```{needget}
* A working knowledge of ROS and the Duckietown development stack
---
* Ability to create clear inline documentation and build a Sphinx web page from it
```

## Why is (inline) documentation important?

Well‑written documentation determines whether code is *usable* or *ignored*.  

A brilliant package that saves developers weeks of effort is worthless if no‑one can discover or understand it.  

Inline docstrings are especially powerful because they live next to the code they explain, are simple to update, and mirror the project’s structure. Tools such as **[Sphinx](https://www.sphinx-doc.org)** then transform those docstrings into polished HTML pages.

```{note}
The book you are reading is *stand‑alone* documentation. Inline documentation complements, rather than replaces, such higher‑level material.
```

Sphinx is the de‑facto standard for Python to parse docstrings and build pages. The magic happens automatically, and success depends on adopting the correct formatting, which we detail below.

---

## What should be documented.. and where?

The short and cheeky answer to this question is "everything, and in the right place". The long answer is the same, in other words: **everything that users may touch should be documented in its closest context.**

For exmaple: 

* **ROS nodes**  
  * Provide an overview of purpose and algorithm.  
  * List configuration parameters, default values, publishers, subscribers, and services.  
  * Document every method: arguments, types, return values.

* **Libraries (`include/` or `src/`)**  
  * Each module and package needs a short description.  
  * All public classes, functions, and methods require docstrings.

* **Repository and package level**  
  * Add an RST file under `docs/packages/` named `<package>.rst` that pulls together node‑ and library‑level docs via Sphinx directives.

  
### Example: ROS node docstring

The documentation of the node itself should _always_ be as a docstring after the class definition. _Do not_ put it, or anything else as a docstring for the `__init__` method. This will not be rendered in the final output.

```python

class CameraNode(DTROS):
    """

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.
    `Picamera <https://picamera.readthedocs.io/>`_ is used for handling 
    the image stream.

    Note that only one :obj:`PiCamera` object should be used at a time. 
    If another node tries to start an instance while this node is running,
    it will likely fail with an `Out of resource` exception.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~framerate (:obj:`float`): The camera image acquisition framerate, default is 30.0 fps
        ~res_w (:obj:`int`): The desired width of the acquired image, default is 640px
        ~res_h (:obj:`int`): The desired height of the acquired image, default is 480px
        ~exposure_mode (:obj:`str`): PiCamera exposure mode

    Publisher:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images

    Service:
        ~set_camera_info:
            Saves a provided camera info to `/data/config/calibrations/camera_intrinsic/HOSTNAME.yaml`.

            input:
                camera_info (obj:`CameraInfo`): The camera information to save

            outputs:
                success (:obj:`bool`): `True` if the call succeeded
                status_message (:obj:`str`): Used to give details about success

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(node_name=node_name,
                                         node_type=NodeType.PERCEPTION)

    [...]    

    def save_camera_info(self, camera_info_msg, filename):
        """Saves intrinsic calibration to file.

            Args:
                camera_info_msg (:obj:`CameraInfo`): Camera Info containing calibration
                filename (:obj:`str`): filename where to save calibration

            Returns:
                :obj:`bool`: whether the camera info was successfully written
        """
        # Convert camera_info_msg and save to a yaml file
        self.log("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        [...]

        self.log("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

```

Start with a high-level description of the function of the node, where it fits in the bigger picture of the package and repository, and what it receives as input(s) and produces as output(s). Feel generous with the description here; remember the latin saying: "_Melius abundare est quam deficere_." ("It is better to have too much than too little"). 

Following the general introduction section, include:
- a section with the arguments needed for initializing the node (the arguments of the `__init__` method), which will almost always be exactly the same as shown. 
- a configuration section with the parameters for the node, their type, a short description, and their default value. 
- additional section describing the subscribers, publishers and services, in this order. 

If the node has no subscribers, as e.g., the camera node, skip this section. 

```{note}
Note the specific way of structuring the documentation of the service.
```

Every method should be documented as a docstring immediately after the function definition (as the `save_camera_info` example). Add a short description of the method, the arguments it expects, and the return value (when applicable).

### Example: library docstring 

Libraries should be documented similarly to nodes. However, when documenting libraries, it is important to actually invoke the Sphinx commands for documenting particular objects in the `__init__.py` file. Furthermore, this file should contain a description of the package itself. Here is an example from the `line_detector` library's `__init__.py` file:

```python
"""

    line_detector
    -------------

    The ``line_detector`` library packages classes and tools for handling line section extraction from images. The
    main functionality is in the :py:class:`LineDetector` class. :py:class:`Detections` is the output data class for
    the results of a call to :py:class:`LineDetector`, and :py:class:`ColorRange` is used to specify the color ranges
    in which :py:class:`LineDetector` is looking for line segments.

    There are two plotting utilities also included: :py:func:`plotMaps` and :py:func:`plotSegments`

    .. autoclass:: line_detector.Detections

    .. autoclass:: line_detector.ColorRange

    .. autoclass:: line_detector.LineDetector

    .. autofunction:: line_detector.plotMaps

    .. autofunction:: line_detector.plotSegments


"""
```

The documentation describes the library and its elements prior to calling the [Sphinx commands](sphinx_style_guide) to parse these classes and functions.

### Example: ROS package

Similarly, every ROS package needs a documentation file. This should go in the `docs/packages` directory of the project's repository and should be named `package_name.rst`. It should describe the package and then should invoke the Sphinx commands for building the documentation for the individual nodes and libraries. E.g.:

```md
ROS Package: ground\_projection
===============================

.. contents::

The ``ground_projection`` package provides the tools for projecting line segments from an image reference frame to the ground reference frame, as well as a ROS node that implements this functionality. It has been designed to be a part of the lane localization pipeline. Consists of the ROS node :py:class:`nodes.GroundProjectionNode` and the :py:mod:`ground_projection` library.


GroundProjectionNode
--------------------

.. autoclass:: nodes.GroundProjectionNode

Included libraries
------------------

.. automodule:: ground_projection

```

(sphinx_style_guide)=
## Style guide

Sphinx uses commands called _directives_. The basic style of the documentation comes from _reStructuredText_, which is the default plaintext markup language used by Sphinx. 

### Basic styles

- Use `*text*` to italicize the text.
- Use `**text**` to make it in boldface.
- Values, names of variables, errors, messages, etc. should be in grave accent quotes:

    ```md
    ``like that``
    ```

- Sections are created by underlying section title with a punctuation character, at least as long as the text:

    ```md
    What a cool heading
    ===================

    Nice subsection
    ---------------

    A neat subsubsection
    ^^^^^^^^^^^^^^^^^^^^
    ```

- External links can be added like this:

    ```md
        For this, we use `Duckietown <https://duckietown.com/>`_ which is a lifesaver.
    ```

- When describing standard types (like `int`, `float`, etc.) use

    ```md
    :obj:`int`
    ```

- If the type is an object of one of the libraries in the repository, then use the referencing directives from the next section to create hyperlinks. If it is a message, use the message type. If a list, a dictionary, or a tuple, use expressions like `` :obj:`list` of :obj:`float` ``

- Attributes of a class can also be documented. The best practice is to do that for all important attributes and for constants.

### Example: documenting class attributes

```python

    class Foo:
        """Docstring for class Foo."""

        #: Doc comment for class attribute Foo.bar.
        #: It can have multiple lines.
        bar = 1

        flox = 1.5   #: Doc comment for Foo.flox. One line only.

        baz = 2
        """Docstring for class attribute Foo.baz."""

        def __init__(self):
            #: Doc comment for instance attribute qux.
            self.qux = 3

            self.spam = 4
            """Docstring for instance attribute spam."""

```

```{seealso}
Additional examples:
- [reStructuredText basics](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html) 
- [Quick references](https://docutils.sourceforge.io/docs/user/rst/quickref.html)
- [Detailed specifications](https://docutils.sourceforge.io/docs/ref/rst/restructuredtext.html).
```

### Referencing other objects

Links to a different package, node, method, or object are added with:

```md
    :py:mod:`duckietown`
    :py:class:`duckietown.DTROS`
    :py:meth:`duckietown.DTROS.publisher`
    :py:attr:`duckietown.DTROS.switch`
```

All of these refer to the `duckietown` Python package. 

When dealing with nodes things are a bit trickier because they are not a part of a package. To make Sphinx work nicely with ROS nodes, we create a fake package that has them all as classes. Hence, to refer to the `CameraNode`:

```md
    :py:class:`nodes.CameraNode`    
```

### Custom sections

When documenting a node, make use of the following ROS-specific sections: ``Examples``, ``Raises``, ``Configuration``, ``Subscribers``, ``Subscriber``, ``Publishers``, ``Publisher``, ``Services``, ``Service``, ``Fields``, ``inputs``, ``input``, ``outputs``, ``output``. 

To add custom sections edit `docs/config.yaml`.

### Using `autodoc`

We use the [autodoc extension](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html) of Sphinx to automatically create the markup from the docstrings in our Python code using the following directives:

```md

.. automodule:: ground_projection

.. autoclass:: line_detector.ColorRange

.. autofunction:: line_detector.plotMaps

.. automethod:: nodes.CameraNode.save_camera_info

```

More [autodoc details](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html) are available on the Sphinx website.
