const mongoose = require('mongoose')

const lostSchema = new mongoose.Schema({
    fileUrl: { type: String, required: true},
    createdAt: { type: String, required: true },
})

const Lost = mongoose.model("Lost", lostSchema)

module.exports = Lost