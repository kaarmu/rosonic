from setuptools import setup

with open('README.md') as f:
    long_description = f.read()

setup(
    name='rosonic',
    version='0.3.0',
    description='Write pythonic ROS nodes, fast and comfortably',
    author='Kaj Munhoz Arfvidsson',
    author_email='kajarf@kth.se',
    url='https://kaarmu.github.io/rosonic/',
    license='MIT',
    long_description=long_description,
    long_description_content_type='text/markdown',
    python_requires='>=3.8',
    extras_require={
        'docs': ['mkdocs', 'mkdocs-with-pdf', ],
    },
    package_dir={'': 'src'},
    packages=['rosonic'],
    provides=['rosonic'],
)
