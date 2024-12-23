import mongoose from "mongoose";


const courseSchema = new mongoose.Schema({
    title: { type: String, required: true },
    description: { type: String, required: true },
    duration: { type: Number, required: true }, // Duration in hours
    category: { type: String, required: true },
    level: { type: String, required: true },
    instructor: { type: String, required: true },
    price: { type: Number, required: true }
})

const Course = mongoose.model('Course', courseSchema);
export default Course;
