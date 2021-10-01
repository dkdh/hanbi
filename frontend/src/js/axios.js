const axios = require("axios")
const params = require("./../config")

const request = axios.create({
    baseURL: params["ip_server"],
    timeout: 3000
})


module.exports = {
    getLost: async () => {
        const res = await request.get("/api/Lost")
        return res["data"]["data"]
    },
}