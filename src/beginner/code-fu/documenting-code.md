(sec:dt_way_code_docs)=
# Documenting your code

This section provides a comprehensive guide to writing inline documentation to your ROS nodes and libraries. We will discuss both _what_ should be documented and _how_ it should be documented. Head to [](#dt_way_build_docs) for the details about how to then create a human-friendly webpage showing the documentation.

## Basics about inline code documentation

Documentation is a must-do in any software project. As harsh as it sounds, you can write an absolutely revolutionary and beautiful software package that can save anyone who uses it years of their time while simultaneously solving all of humanity's greatest problems. But if people do not know that your code exists, have no idea how to use it, or understanding its intricacies takes too much effort, then you will neither save anyone any time, nor solve any problem. In fact, as far as the world beyond you is concerned, all your work is as good as if never done. Therefore, if you wish your code to be ever used, __document__ it as extensively as possible!

Inline documentation is a pretty handy way of helping people use your software. On one hand, it is right in the place where it is needed: next to the classes and methods that you are documenting. On the hand, it is also much easier to update when you change something in the code: the documentation of your function is typically no more than 20 lines above your change. On the third hand (should you have one) documentation written in this way inherits the structure of your software project, which is a very natural way of organizing it. On the fourth hand (you can borrow someone else's), there are some really nice packages that take your documentation and make it into a beautiful webpage.

```{note}
The documentation that you are currently reading is called _a book_ and exists independent of any code repository.
```

In Duckietown we use Sphinx for building our code documentation. Sphinx is the most popular way of creating code documentation for Python projects. You can find out more on [their webpage](https://www.sphinx-doc.org) and there are a lot of interesting things to read there. Documenting your code is as simple as writing docstrings and the occasional comment. Then, Sphinx takes care of parsing all your docstrings and making a nice webpage for it. However, in order for all this to work nicely, you need to format your documentation in a particular way. We will discuss this later in this page.

## What should be documented and where?

The short answer to this question is "everything and in the right place". The long answer is the same.

Every ROS node should be documented, meaning a general description of what it is for, what it does, and how it does it. Additionally, all its parameters, publishers, subscribers, and services need to be described. The default values for the parameters should be also added in the documentation. Every method that your node's class has should also be documented, including the arguments to the method and the returned object (if there is such), as well as their types.

Every library in your `include` directory should also be documented. That again means, every class, every method, every function. Additionally, the library itself, and its modules should have a short description too.

Finally, your whole repository, and every single package should also be documented.

Let's start from a node. Here is a sample for the camera node:

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

The documentation of the node itself should _always_ be as a docstring after the class definition. _Do not_ put it, or anything else as a docstring for the `__init__` method. This will not be rendered in the final output.

The documentation of the node should start with a general description about the node, its purpose, where it fits in the bigger picture of the package and repository, etc. Feel generous with the description here. Then there is a section with the arguments needed for initializing the node (the arguments of the `__init__` method) which will almost always be exactly the same as shown. After that there is a configuration section where you should put all the parameters for the node, their type, a short description, and their default value, as shown.

This is then followed by Subscribers, Publishers and Services, in this order. If the node has no Subscribers, for example as the camera node, then you don't need to add this section. Note the specific way of structuring the documentation of the service!

Then, every method should be documented as a docstring immediately after the function definition (as the `save_camera_info` example). Again, add a short description of the method, as well as the arguments it expects and the return value (should such exist).

Libraries should be documented in a similar way. However, when documenting libraries, it is important to actually invoke the Sphinx commands for documenting particular objects in the `__init__.py` file. Furthermore, this file should contain a description of the package itself. Here's an example from the `line_detector` library's `__init__.py` file:

```python
"""

    line_detector
    -------------

    The ``line_detector`` library packages classes and tools for handling line section extraction from images. The
    main functionality is in the :py:class:`LineDetector` class. :py:class:`Detections` is the output data class for
    the results of a call to :py:class:`LineDetector`, and :py:class:`ColorRange` is used to specify the colour ranges
    in which :py:class:`LineDetector` is looking for line segments.

    There are two plotting utilities also included: :py:func:`plotMaps` and :py:func:`plotSegments`

    .. autoclass:: line_detector.Detections

    .. autoclass:: line_detector.ColorRange

    .. autoclass:: line_detector.LineDetector

    .. autofunction:: line_detector.plotMaps

    .. autofunction:: line_detector.plotSegments


"""
```

You can see that it describes the library and its elements, and then uses the Sphinx commands which will parse these classes and functions and will add their documentation to this page. You can find more details about these functions in [](#sphinx_style_guide).

Similarly, every ROS package needs a documentation file. This should go in the `docs/packages` directory of your repository and should be named `package_name.rst`. It should describe the package and then should invoke the Sphinx commands for building the documentation for the individual nodes and libraries. See the following example:

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

You probably noticed the plethora of funky commands in the above examples. These are called _directives_, and we'll now take a closer look at them. The basic style of the documentation comes from reStructuredText, which is the default plaintext markup language used by Sphinx. The rest are Sphinx directives which Sphinx then replaces with markup which it creates from your docstrings.

### Basic styles

- You can use `*text*` to italicize the text.
- You can use `**text**` to make it in boldface.
- Values, names of variables, errors, messages, etc. should be in grave accent quotes:

    ```md
    ``like that``
    ```

- Section are created by underlying section title with a punctuation character, at least as long as the text:

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
        For this, we use `Picamera <https://picamera.readthedocs.io/>`_ which is an external library.
    ```

- When describing standard types (like `int`, `float`, etc.) use

    ```md
    :obj:`int`
    ```

- If the type is an object of one of the libraries in the repository, then use the referencing directives from the next section in order to create hyperlinks. If it is a message, use the message type. If a list, a dictionary, or a tuple, you can use expressions like `` :obj:`list` of :obj:`float` ``

- Attributes of a class can also be documented. We recommend that you do that for all important attributes and for constants. Here are examples of the various ways you can document attributes:

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
You can find more examples with reStructuredText [here](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html) and [here](https://docutils.sourceforge.io/docs/user/rst/quickref.html), and detailed specification [here](https://docutils.sourceforge.io/docs/ref/rst/restructuredtext.html).
```
### Referencing other objects

You can add a link to a different package, node, method, or object like that:

```md
    :py:mod:`duckietown`
    :py:class:`duckietown.DTROS`
    :py:meth:`duckietown.DTROS.publisher`
    :py:attr:`duckietown.DTROS.switch`
```

All of these refer to the `duckietown` Python package. When dealing will nodes, things are a bit trickier, because they are not a part of a package. However, in order to make Sphinx work nicely with ROS nodes, we create a fake package that has them all as classes. Hence, if you want to refer to the `CameraNode`, you can do it like that:

```md
    :py:class:`nodes.CameraNode`    
```

```{warning}
We are considering replacing `nodes` with the repository name, so keep in mind this might change soon.
```

### Custom sections

When documenting a node, you can (and you should) make use of the following ROS-specific sections: ``Examples``, ``Raises``, ``Configuration``, ``Subscribers``, ``Subscriber``, ``Publishers``, ``Publisher``, ``Services``, ``Service``, ``Fields``, ``inputs``, ``input``, ``outputs``, ``output``. If you need other custom sections you can add them in the `docs/config.yaml` file in your repository.

### Using autodoc

We use the [autodoc extension](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html) of Sphinx in order to automatically create the markup from the docstrings in our Python code. In particular, you can use the following directives:

```md

.. automodule:: ground_projection

.. autoclass:: line_detector.ColorRange

.. autofunction:: line_detector.plotMaps

.. automethod:: nodes.CameraNode.save_camera_info

```

You can find more details [here](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html).
