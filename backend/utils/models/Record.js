const mongoose = require('mongoose')

const recordSchema = new mongoose.Schema({
    fileUrl: { type: String, required: true},
    createdAt: { type: String, required: true },
})

const Record = mongoose.model("Record", recordSchema)

module.exports = Record