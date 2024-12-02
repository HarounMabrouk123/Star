import jwt from "jsonwebtoken";
import bcrypt from "bcryptjs";
import User from "../models/userModel.js";
import Mentor from "../models/mentorModel.js";
import dotenv from "dotenv";
dotenv.config();

// Mentor Registration
export const registerMentor = async (req, res) => {
  const { name, email, password, specializations, experience } = req.body;

  try {
    // Check if the user already exists
    const existingUser = await User.findOne({ email });
    if (existingUser) {
      return res.status(400).json({ message: "User already exists" });
    }

    // Hash the password
    const hashedPassword = await bcrypt.hash(password, 12);

    // Create mentor details
    const mentor = await Mentor.create({ specializations, experience });
    const mentorDetails = mentor._id;

    // Create the user
    const user = await User.create({
      name,
      email,
      password: hashedPassword,
      role: "Mentor",
      mentorDetails,
    });

    // Generate JWT Token
    const token = jwt.sign({ id: user._id, role: user.role }, process.env.JWT_SECRET, {
      expiresIn: "1h",
    });

    res.status(201).json({ user, token });
  } catch (error) {
    console.error(error.message);
    res.status(500).json({ message: "Something went wrong", error: error.message });
  }
};