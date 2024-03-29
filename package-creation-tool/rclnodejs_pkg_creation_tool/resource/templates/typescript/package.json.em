{
  "name": "@(name)",
  "version": "@(version)",
  "description": "@(description)",
  "keywords": [],
  "author": "",
  "license": "@(license)",
  "scripts": {
  	"build": "tsc",
    "test": "echo \"Error: no test specified\" && exit 1"
  },
  "bin": {
    "generate-messages": "node_modules/.bin/generate-ros-messages"
  },
  "dependencies": {
  },
  "devDependencies": {
    "@@types/node": "^18.15.0",
    "nodemon": "^2.0.21",
    "ts-node": "^10.9.0",
    "typescript": "^4.9.0"
  }

}
