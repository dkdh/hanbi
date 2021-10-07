const NODE_ENV = process.env.NODE_ENV || "developments"
const params = {
    "developments": {
        origin: ["http://j5a102.p.ssafy.io:8080"]
    },
}
module.exports = params[NODE_ENV]