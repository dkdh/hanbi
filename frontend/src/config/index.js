const NODE_ENV = process.env.NODE_ENV || "development"
const params = {
    "development": {
        "ip_server": "http://j5a102.p.ssafy.io:3000"
    },
}
console.log(params[NODE_ENV])
module.exports = params[NODE_ENV]