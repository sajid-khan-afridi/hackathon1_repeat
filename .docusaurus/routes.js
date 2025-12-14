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
    component: ComponentCreator('/hackathon1_repeat/docs', 'fdd'),
    routes: [
      {
        path: '/hackathon1_repeat/docs',
        component: ComponentCreator('/hackathon1_repeat/docs', 'c1d'),
        routes: [
          {
            path: '/hackathon1_repeat/docs/tags',
            component: ComponentCreator('/hackathon1_repeat/docs/tags', '2dc'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/autonomous-systems',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/autonomous-systems', 'f1d'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/autonomy',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/autonomy', '7f2'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/beginner',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/beginner', 'd6d'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/bipedal',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/bipedal', 'c42'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/communication',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/communication', '8cb'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/computer-vision',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/computer-vision', '7d0'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/control',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/control', '438'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/fundamentals',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/fundamentals', '62c'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/humanoid',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/humanoid', 'a36'),
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
            path: '/hackathon1_repeat/docs/tags/kinematics',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/kinematics', '367'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/launch',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/launch', 'ef9'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/lifecycle',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/lifecycle', 'cd5'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/locomotion',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/locomotion', '136'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/manipulators',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/manipulators', '826'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/mathematics',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/mathematics', '730'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/mobile-robots',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/mobile-robots', 'aec'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/modeling',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/modeling', '86b'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/navigation',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/navigation', 'e2f'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/nodes',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/nodes', '717'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/parameters',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/parameters', 'da8'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/path-planning',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/path-planning', 'eea'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/perception',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/perception', 'e1b'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/physics',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/physics', 'cf2'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/robotics',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/robotics', '221'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/ros-2',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/ros-2', '382'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/ros-2-bridge',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/ros-2-bridge', '89c'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/sensor-fusion',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/sensor-fusion', 'f0e'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/sensors',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/sensors', 'f4a'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/services',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/services', '18c'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/setup',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/setup', '5f1'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/simulation',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/simulation', '40f'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/topics',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/topics', 'b70'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs/tags/urdf',
            component: ComponentCreator('/hackathon1_repeat/docs/tags/urdf', 'b16'),
            exact: true
          },
          {
            path: '/hackathon1_repeat/docs',
            component: ComponentCreator('/hackathon1_repeat/docs', '8c4'),
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
                path: '/hackathon1_repeat/docs/contributing/chapter-authoring',
                component: ComponentCreator('/hackathon1_repeat/docs/contributing/chapter-authoring', '331'),
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
                path: '/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle',
                component: ComponentCreator('/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle', 'c41'),
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
                path: '/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-2-topics-services',
                component: ComponentCreator('/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-2-topics-services', '485'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-3-parameters-launch',
                component: ComponentCreator('/hackathon1_repeat/docs/module-1-ros2-fundamentals/chapter-3-parameters-launch', 'f85'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-2-isaac-sim/chapter-1-introduction',
                component: ComponentCreator('/hackathon1_repeat/docs/module-2-isaac-sim/chapter-1-introduction', 'dee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-2-isaac-sim/chapter-1-introduction-comprehensive',
                component: ComponentCreator('/hackathon1_repeat/docs/module-2-isaac-sim/chapter-1-introduction-comprehensive', 'be5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-2-isaac-sim/chapter-2-urdf',
                component: ComponentCreator('/hackathon1_repeat/docs/module-2-isaac-sim/chapter-2-urdf', '7ed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-2-isaac-sim/chapter-3-sensors',
                component: ComponentCreator('/hackathon1_repeat/docs/module-2-isaac-sim/chapter-3-sensors', '7eb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-3-applications/chapter-1-kinematics',
                component: ComponentCreator('/hackathon1_repeat/docs/module-3-applications/chapter-1-kinematics', '62f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-3-applications/chapter-2-navigation',
                component: ComponentCreator('/hackathon1_repeat/docs/module-3-applications/chapter-2-navigation', 'de6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-3-applications/chapter-3-perception',
                component: ComponentCreator('/hackathon1_repeat/docs/module-3-applications/chapter-3-perception', '28d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon1_repeat/docs/module-3-applications/chapter-4-humanoid',
                component: ComponentCreator('/hackathon1_repeat/docs/module-3-applications/chapter-4-humanoid', '15d'),
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
