---
layout: archive
title: "CV"
permalink: /cv/
author_profile: true
redirect_from:
  - /resume
---

{% include base_path %}

Education
======
* B.S. in Computer Science, Cornell University, Expected [Year]

Research Experience
======
* [Add your research experience here]
  * Institution/Lab Name
  * Project description
  * Advisor: Professor Name

Work Experience
======
* [Add your work experience here]
  * Company/Organization
  * Role and responsibilities
  * Duration

Skills
======
* Programming Languages
  * Python
  * C/C++
  * [Add more languages]
* Machine Learning/AI
  * [Add relevant skills]
* Robotics
  * [Add relevant skills]
* Tools & Frameworks
  * [Add tools you use]

Publications
======
  <ul>{% for post in site.publications reversed %}
    {% include archive-single-cv.html %}
  {% endfor %}</ul>

Research Projects
======
  <ul>{% for post in site.portfolio reversed %}
    {% include archive-single-cv.html %}
  {% endfor %}</ul>

Teaching
======
  <ul>{% for post in site.teaching reversed %}
    {% include archive-single-cv.html %}
  {% endfor %}</ul>

Awards and Honors
======
* [Add awards and honors here]

Service and Leadership
======
* [Add service and leadership activities here]
