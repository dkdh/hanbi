const NODE_ENV = process.env.NODE_ENV || "development"
const params = {
    "development": {
        "ip_server": "http://localhost:3000"
    },
}
console.log(params[NODE_ENV])
module.exports = params[NODE_ENV]