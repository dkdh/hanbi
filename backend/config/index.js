const NODE_ENV = process.env.NODE_ENV || "developments"
const params = {
    "developments": {
        origin: ["http://localhost:8080", "http://localhost:8079", "http://localhost:8081"]
    },
}
module.exports = params[NODE_ENV]