import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import LandingPage from "./pages/LandingPage";
import Login from "./pages/Login";
import Tregister from "./pages/Tregister";
import Mregister from "./pages/Mregister";
import UserProfile from "./pages/TeacherProfile";
import CoursesPage from "./pages/CoursesPage";
import SingleCoursePage from "./pages/SingleCoursePage";
import MentorProfile from "./pages/MentorProfile";
import BrainMate from "./pages/BrainMate";
import Contact from "./pages/Contact";
import Tools from "./pages/TeacherToolsDashboard"
import TeacherToolsDashboard from "./pages/TeacherToolsDashboard";



export default function App() {
  return (
    <Router>
        <Routes>
          <Route path="/" element={<LandingPage />} />
          <Route path="/login" element={<Login />} />
          <Route path="/RegisterAsT" element={<Tregister />} />
          <Route path="/RegisterAsM" element={<Mregister />} />
          <Route path="/profile" element={<UserProfile />} />
          <Route path="/mentor" element={<MentorProfile />} />
          <Route path="/courses" element={<CoursesPage />} />
          <Route path="/courses/1" element={<SingleCoursePage />} />
          <Route path="/BrainMate" element={<BrainMate />} />
          <Route path="/contact" element={<Contact />} />
          <Route path="/tools" element={<TeacherToolsDashboard />} />
        </Routes>
    </Router>
  );
}

