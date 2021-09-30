const NODE_ENV = process.env.NODE_ENV || "developments"
const params = {
    "developments": {
        origin: ["http://localhost:8080"]
    },
}
module.exports = params[NODE_ENV]