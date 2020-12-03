---
title: "SquatDetector"
subtitle: 
date:  2020-10-8T10:38:23+02:00
draft: true
author: "Leon"
authorLink: ""
description: ""
license: ""
images: []

tags: []
categories: []
featuredImage: ""
featuredImagePreview: ""

hiddenFromHomePage: false
hiddenFromSearch: false
twemoji: false
lightgallery: true
ruby: true
fraction: true
fontawesome: true
linkToMarkdown: true
rssFullText: false

toc:
  enable: true
  auto: true
code:
  copy: true
  # ...
math:
  enable: true
  # ...
mapbox:
  accessToken: ""
  # ...
share:
  enable: true
  # ...
comment:
  enable: true
  # ...
library:
  css:
    # someCSS = "some.css"
    # located in "assets/"
    # Or
    # someCSS = "https://cdn.example.com/some.css"
  js:
    # someJS = "some.js"
    # located in "assets/"
    # Or
    # someJS = "https://cdn.example.com/some.js"
seo:
  images: []
  # ...
---


# This is a Test of all the functionalities of the snippets i created to help me produce the content



{{<admonition type=question title="Does work properly">}}
Are all blocks working as there are intedet or are there any issues regarding the snipped implemenation such as:
----
  - Wrong spelling
  - wrong anything
    - okay lol this is a sub point!!



{{</admonition>}}

$$\begin{bmatrix}   a & b \\\\   c & d \end{bmatrix}$$


$$\begin{bmatrix}   a & b \\\\   c & d \end{bmatrix}$$

$$\begin{bmatrix} \dot{x} \\\\ \ddot{x} \\\\ \dot{\phi} \\\\ \ddot{\phi} \end{bmatrix} =\begin{bmatrix} 0 & 1 & 0 & 0 \\\\ 0 & \frac{-\left(I+m l^{2}\right) b}{I(M+m)+M m l^{2}} & \frac{m^{2} g l^{2}}{I(M+m)+M m l^{2}} & 0 \\\\ 0 & 0 & 0 & 1 \\\\ 0 & \frac{-m l b}{I(M+m)+M m l^{2}} & \frac{m g l(M+m)}{I(M+m)+M m l^{2}} & 0 \end{bmatrix}$$

$$\begin{bmatrix}{c}
x \\\\
\dot{x} \\\\
\phi \\\\
\dot{\phi}
\end{bmatrix}$$

```js
grunt.initConfig({
  assemble: {
    options: {
      assets: 'docs/assets',
      data: 'src/data/*.{json,yml}',
      helpers: 'src/custom-helpers.js',
      partials: ['src/partials/**/*.{hbs,md}']
    },
    pages: {
      options: {
        layout: 'default.hbs'
      },
      files: {
        './': ['src/templates/pages/index.hbs']
      }
    }
  }
};
```


