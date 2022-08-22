from setuptools import setup

with open('README.md') as f:
    long_description = f.read()

setup(
    name='rosonic',
    version='0.2',
    description='Write pythonic ROS nodes, fast and comfortably',
    author='Kaj Munhoz Arfvidsson',
    author_email='kajarf@kth.se',
    url='https://github.com/kaarmu/rosonic',
    license='MIT',
    long_description=long_description,
    long_description_content_type='text/markdown',
    python_requires='>=2.7',
    install_requires=['rospy>=1.14'],
    extras_require={
        'docs': ['mkdocs', 'mkdocs-with-pdf', ],
    },
    package_dir={'': 'src'},
    packages=['rosonic'],
    provides=['rosonic'],
)
