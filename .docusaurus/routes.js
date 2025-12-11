import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/hackathon1_repeat/markdown-page',
    component: ComponentCreator('/hackathon1_repeat/markdown-page', 'c23'),
    exact: true
  },
  {
    path: '/hackathon1_repeat/docs',
    component: ComponentCreator('/hackathon1_repeat/docs', '762'),
    routes: [
      {
        path: '/hackathon1_repeat/docs',
        component: ComponentCreator('/hackathon1_repeat/docs', 'aae'),
        routes: [
          {
            path: '/hackathon1_repeat/docs/tags',
            component: ComponentCreator('/hackathon1_repeat/docs/tags', '2dc'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/beginner',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/beginner', 'd6d'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/communication',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/communication', '8cb'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/fundamentals',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/fundamentals', '62c'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/introduction',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/introduction', '207'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/isaac-sim',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/isaac-sim', '939'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/ros-2',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/ros-2', '382'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/simulation',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/simulation', '40f'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs',
            component: ComponentCreator('/hackathon1_repeat/docs', 'd44'),
            routes: [
              {
                path: '/hackathon1_repeat/docs/category/module-1-ros-2-fundamentals',
                component: ComponentCreator('/hackathon1_repeat/docs/category/module-1-ros-2-fundamentals', '78c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/category/module-2-nvidia-isaac-sim',
                component: ComponentCreator('/hackathon1_repeat/docs/category/module-2-nvidia-isaac-sim', '614'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/github-pages-deployment-research',
                component: ComponentCreator('/hackathon1_repeat/docs/github-pages-deployment-research', '9b6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/intro',
                component: ComponentCreator('/hackathon1_repeat/docs/intro', '589'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-1-publishers',
                component: ComponentCreator('/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-1-publishers', '7e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-2-isaac-sim/chapter-1-introduction',
                component: ComponentCreator('/hackathon1_repeat/docs/module-2-isaac-sim/chapter-1-introduction', 'dee'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/hackathon1_repeat/',
    component: ComponentCreator('/hackathon1_repeat/', 'a9d'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
