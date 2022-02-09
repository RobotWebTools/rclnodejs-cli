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
    "@@types/node": "ts4.5",
    "nodemon": "^2.0.15",
    "ts-node": "^10.5.0",
    "typescript": "^4.5.0"
  }

}
