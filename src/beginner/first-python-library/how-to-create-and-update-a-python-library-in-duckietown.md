```{seo}
:description: Learn how to create and release a Python library using the Duckietown template-library, including project setup, testing, and version management.
:keywords: Duckietown, Python library, template-library, Sphinx, CircleCI, CodeCov, Docker, bumpversion, Black
```

(python-library)=
# Beginner - Python Library

In this section you will learn how to create a Python library following the Duckietown template.

(generate-repo)=
## Step 1: Get the Duckietown library template

A boilerplate is provided by the [library template repository](https://github.com/duckietown/template-library).

Click **Use this template** → **Create a new repository**.

```{figure} ../../_images/beginner/github_use_template.jpg
:width: 70%
:name: fig:library-github-use-template
:alt: GitHub template use button
:align: center

Use the library template repository on GitHub.
```

```{figure} ../../_images/beginner/library/create-repo-from-template.png
:width: 90%
:name: fig:library-create-repo-from-template
:alt: Creating repository from template
:align: center

Creating a repository from template.
```

Clone the new repository (which is `my-library` in this example, but any name will do):

```bash
git clone https://github.com/YOUR_USERNAME/my-library
cd my-library
```

(template-features)=
## Features of the library template

* Unit‑tests with [Nose](https://nose.readthedocs.io/en/latest/)  
* Docker‑based local build/test  
* [CircleCI](https://circleci.com/gh/duckietown) integration  
* [CodeCov](https://codecov.io/gh/duckietown) coverage reports  
* Integration with [Sphinx](https://www.sphinx-doc.org/en/master/) to build code docs. (So far, only built locally.)
* [Jupyter](https://jupyter.org/) notebooks, which are run also in CircleCI as tests.
* Version bump using [Bumpversion](https://github.com/peritus/bumpversion).
* Code formatting using [Black](https://github.com/psf/black).
* Command-line program for using the library.

## Anatomy of the library template

This repository describes a library called "`duckietown_pondcleaner`" and there is one command-line tool called `dt-pc-demo.`

### Meta-files


* `.gitignore`: Files ignore by Git.


* `.dtproject`: Enables the project to be built and used by `dts devel` tools


* `.bumpversion.cfg`: Configuration for bumpversion


* `Makefile`: Build tools configuration with Make


### Python packaging


* `requirements.txt`: Contains the *pinned* versions of your requirement that
 are used to run tests.
 
* `MANIFEST.in`: Deselects the tests to be included in the egg.


* `setup.py`: Contains meta information, definition of the scripts, and
 the dependencies information. 


### Python code


* `src/` - This is the path that you should set as "sources root" in your tool


* `src/duckietown_pondcleaner`: Contains the code.


* `src/duckietown_pondcleaner/__init__.py`: Contains the `__version__` library.


* `src/duckietown_pondcleaner_tests`: Contains the tests - not included in the egg.


### Docker testing


These are files to build and run a testing container.


* `.dockerignore`: Describes what files go in the docker container.


* `Dockerfile`: The build configuration for the software image


### Sphinx


* `src/conf.py`: Sphinx settings


* `src/index.rst`: Sphinx main file


* `src/duckietown_pondcleaner/index.rst`: Documentation for the package




### Coverage


* `.coveragerc`: Options for code coverage.




### Notebooks


* `notebooks`: Notebooks that are run also as a test.


* `notebooks-extra`: Other notebooks (not run as test)


* `notebooks/*.ipynb`: The notebooks themselves.

## Step 2: Creating your Library

1. Clone your repository  
2. Place Python packages in `src/`  
3. List Python dependencies in `dependencies.txt`  
4. Update `setup.py` metadata  
5. Clean leftover references:  
   ```bash
   grep -r . pondcleaner
   ```  
6. Update branch names in `README.md`  

### Admin setup

1. Enable CircleCI, ensure build passes  
2. Enable CodeCov, set `CODECOV_TOKEN` in CircleCI environment  

(template-utils)=
## Step 3: Using the library template utilities

### Test the code

Test the code using Docker by:

    make test-docker
  
This runs the test using a Docker container built from scratch
with the pinned dependencies in `requirements.txt`.

This is equivalent to what is run on CircleCI.

To run the tests locally, use:

    make test


```{note}
To run the tests you will need to have installed the libraries listed in the file `requirements.txt` on your computer.

For that we assume you have already set up a Python virtual environment.

To use a Python virtual environment you will need to `pip install virtualenv` then `virtualenv duckietown` then `source
duckietown/bin/activate`. In order to install the requirements to run the test do `pip install -r requirements.txt`.
```


### Development

In the same virtual environment as above run:

    python setup.py develop
  
This will install the library in an editable way (rather than copying the sources somewhere else).

To skip installing dependencies, use `--no-deps`:


    python setup.py develop --no-deps
  
For example, this is done in the Dockerfile so that we know we are only using the dependencies in `requirements.txt` with the exact pinned version.

### Adding tests

To define new tests, add files with the name `test_*py` in the
package `duckietown_podcleaner_tests`. The name is important.


```{tip}
Make sure that the tests are actually run by looking at the coverage results.
```


### Using the notebooks

Always clean the notebooks before committing them:

    make -C notebooks cleanup
            
```{warning}
If you do not think you can be diligent about this, then add the notebooks using Git LFS.
```

(release-library)=
## Step 4: Releasing a new library version


### Updating the version

The first step is to change the version and tag the repository.

**DO NOT** change the version manually; use the CLI tool `bumpversion` instead.

The tool can be called by:


    make bump    # bump the version, tag the tree
  
If you need to include the version in a new file, list it inside the file `.bumpversion.cfg` using the
syntax `[bumpversion:file: &lt;FILE_PATH &gt;]`.


### Releasing the package


The next step is to upload the package to PyPy, by using [twine](https://pypi.org/project/twine/):

    make upload  # upload to PyPI

```{note}
This step, requires admin permissions on PyPy.
```

<!--
(python-library)=
# Beginner - Python Library

In this section, you will create your own Python library following the Duckietown template.

(generate-repo)=
## Step 1: Get the Duckietown library template

A boilerplate is provided by the [library template repository](https://github.com/duckietown/template-library).

The repository contains a lot of files, but do not worry, we will analyze them one by one.
Click on the button that reads "Use this template" and then choose
"Create a new repository" from the dropdown menu.

```{figure} ../../_images/beginner/github_use_template.jpg
:width: 70%
:name: fig:library-github-use-template

Use template repository on GitHub.
```

This will take you to a page that looks like the following:

```{figure} ../../_images/beginner/library/create-repo-from-template.png
:width: 90%
:name: fig:library-create-repo-from-template

Creating a repository from template.
```

Pick a name for your repository (say `my-library`) and press the button *Create repository from template*.

```{note}
You can replace `my-library` with the name of the repository that you prefer.
```

This will create a new repository and copy everything from the repository `template-library` to your new repository. You can now open a terminal and clone your newly created repository.

    git clone https://github.com/YOUR_USERNAME/my-library
    cd my-library

```{attention}
Replace `YOUR_USERNAME` in the link above with your GitHub username.
```

(template-features)=
## Features of the library template

We have the following features in our new library:

* Unit-tests using [Nose](https://nose.readthedocs.io/en/latest/).
* Building/testing in Docker environment locally.
* Integration with [CircleCI](https://circleci.com/gh/duckietown) for automated testing.
* Integration with [CodeCov](https://codecov.io/gh/duckietown) for displaying coverage result.
* Integration with [Sphinx](https://www.sphinx-doc.org/en/master/) to build code docs. (So far, only built locally.)
* [Jupyter](https://jupyter.org/) notebooks, which are run also in CircleCI as tests.
* Version bump using [Bumpversion](https://github.com/peritus/bumpversion).
* Code formatting using [Black](https://github.com/psf/black).
* Command-line program for using the library.


## Anatomy of the library template

This repository describes a library called "`duckietown_pondcleaner`" and there is one command-line tool called `dt-pc-demo.`

### Meta-files

* `.gitignore`: Files ignore by Git.

* `.dtproject`: Enables the project to be built and used by `dts devel` tools

* `.bumpversion.cfg`: Configuration for bumpversion

* `Makefile`: Build tools configuration with Make

### Python packaging

* `requirements.txt`: Contains the *pinned* versions of your requirement that
  are used to run tests. 
   
* `MANIFEST.in`: Deselects the tests to be included in the egg.
 
* `setup.py`: Contains meta information, definition of the scripts, and 
  the dependencies information.  

### Python code

* `src/` - This is the path that you should set as "sources root" in your tool

* `src/duckietown_pondcleaner`: Contains the code.

* `src/duckietown_pondcleaner/__init__.py`: Contains the `__version__` library.

* `src/duckietown_pondcleaner_tests`: Contains the tests - not included in the egg.

### Docker testing

These are files to build and run a testing container.

* `.dockerignore`: Describes what files go in the docker container.

* `Dockerfile`: The build configuration for the software image

### Sphinx

* `src/conf.py`: Sphinx settings

* `src/index.rst`: Sphinx main file

* `src/duckietown_pondcleaner/index.rst`: Documentation for the package


### Coverage

* `.coveragerc`: Options for code coverage. 


### Notebooks

* `notebooks`: Notebooks that are run also as a test.

* `notebooks-extra`: Other notebooks (not run as test)

* `notebooks/*.ipynb`: The notebooks themselves.


## Step 2: Creating your Library 

Using the repo you have already created:

- Clone the newly created repository;
- Place your Python packages inside `src/`;
- List the python dependencies in the file `dependencies.txt`;
- Update the appropriate section in the file `setup.py`;

Make sure that there are no other remains:

    grep -r . pondcleaner

Update the branch names in `README.md`.

### Other set up (for admins)

The following are necessary steps for admins to do:

1. Activate on CircleCI. Make one build successful.

2. Activate on CodeCov. Get the `CODECOV_TOKEN`. Put this token in CircleCI environment. 


(template-utils)=
## Step 3: Using the library template utilities

### Test the code

Test the code using Docker by:

    make test-docker
    
This runs the test using a Docker container built from scratch
with the pinned dependencies in `requirements.txt`.
This is equivalent to what is run on CircleCI.

To run the tests natively on your pc, use:

    make test

```{note}

To run the tests you will need to have installed the libraries listed in the file `requirements.txt` on your computer.

For that we assume you have already set up a Python virtual environment.

To use a Python virtual environment you will need to `pip install virtualenv` then `virtualenv duckietown` then `source 
duckietown/bin/activate`. In order to install the requirements to run the test do `pip install -r requirements.txt`.
```

### Development

In the same virtual environment as above run:

    python setup.py develop 
    
This will install the library in an editable way (rather than copying the sources somewhere else).

If you don't want to install the deps, do:

    python setup.py develop  --no-deps
    
For example, this is done in the Dockerfile so that 
we know we are only using the dependencies in `requirements.txt` with the 
exact pinned version.

 
### Adding tests

To add another tests, add files with the name `test_*py` in the 
package `duckietown_podcleaner_tests`. The name is important.

```{tip} 
Make sure that the tests are actually run by looking at the coverage results.
```

### Using the notebooks

Always clean the notebooks before committing them:

    make -C notebooks cleanup
              
```{warning}
If you don't think you can be diligent about this, then add the notebooks using Git LFS.
```

(release-library)=
## Step 4: Releasing a new version of your library

### Updating the version

The first step is to change the version and tag the repo. 
**DO NOT** change the version manually; use the CLI tool `bumpversion` instead.

The tool can be called by:

    make bump    # bump the version, tag the tree
    
If you need to include the version in a new file, list it inside the file `.bumpversion.cfg` using the
syntax `[bumpversion:file: &lt;FILE_PATH &gt;]`.

### Releasing the package

The next step is to upload the package to PyPy. 
We use [twine](https://pypi.org/project/twine/). Invoke it using: 
 
    make upload  # upload to PyPI

For this step, you need to have admin permissions on PyPy.
-->