const axios = require("axios")

const request = axios.create({
    baseURL: "http://localhost:3000",
    timeout: 3000
})


module.exports = {
    getLost: async () => {
        const res = await request.get("/api/Lost")
        return res["data"]["data"]
    },
}