import React, { createContext, useState, useEffect, ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface User {
  id: string;
  email: string;
  created_at: string;
  last_login: string | null;
}

export interface UserProfile {
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
  language_preference: string;
  python_level?: string;
  ros2_level?: string;
  gpu_available?: string;
  hardware_tier?: string;
  primary_goal?: string;
}

export interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string, rememberMe?: boolean) => Promise<void>;
  signup: (email: string, password: string, profile: UserProfile) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (updates: Partial<UserProfile>) => Promise<void>;
  refreshProfile: () => Promise<void>;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  
  // ✅ FIXED: Hardcoded Render URL for production stability
  const API_BASE_URL = 'https://physical-ai-auth-backend.onrender.com';

  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  useEffect(() => {
    const loadAuthState = () => {
      const token = localStorage.getItem('auth_token');
      const storedUser = localStorage.getItem('user');
      const storedProfile = localStorage.getItem('profile');

      if (token && storedUser && storedProfile) {
        try {
          setUser(JSON.parse(storedUser));
          setProfile(JSON.parse(storedProfile));
        } catch (error) {
          localStorage.clear();
        }
      }
      setIsLoading(false);
    };
    loadAuthState();
  }, []);

  const signup = async (email: string, password: string, profileData: UserProfile) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        email,
        password,
        software_background: profileData.software_background,
        hardware_experience: profileData.hardware_experience,
        language_preference: profileData.language_preference || 'English',
        python_level: profileData.python_level,
        ros2_level: profileData.ros2_level,
        gpu_available: profileData.gpu_available,
        hardware_tier: profileData.hardware_tier,
        primary_goal: profileData.primary_goal,
      }),
    });

    const data = await response.json();

    if (!response.ok) {
      // ✅ UNIVERSAL ERROR CHECK: Detects 'already exists' in any field or status code
      const errorMessage = (data.message || data.code || JSON.stringify(data)).toLowerCase();
      const isDuplicate = 
        errorMessage.includes('already') || 
        errorMessage.includes('exists') || 
        response.status === 422;

      if (isDuplicate) {
        throw new Error('This email is already registered. Please login instead.');
      }
      throw new Error(data.message || 'Signup failed due to server error');
    }

    localStorage.setItem('auth_token', data.session?.token || data.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.user.profile));

    setUser(data.user);
    setProfile(data.user.profile);
    localStorage.setItem('isLoggedIn', 'true');
  };

  const login = async (email: string, password: string, rememberMe = false) => {
    // ✅ FIXED: Correct Better-Auth sign-in endpoint
    const response = await fetch(`${API_BASE_URL}/api/auth/sign-in/email`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password, rememberMe }),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.message || data.error?.message || 'Login failed. Check your credentials.');
    }

    localStorage.setItem('auth_token', data.token || data.session?.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile || data.user.profile));

    setUser(data.user);
    setProfile(data.profile || data.user.profile);
    localStorage.setItem('isLoggedIn', 'true');
  };

  const logout = async () => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      try {
        await fetch(`${API_BASE_URL}/api/auth/logout`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${token}`,
          },
        });
      } catch (error) {
        console.error('Logout API call failed:', error);
      }
    }
    localStorage.clear();
    setUser(null);
    setProfile(null);
    window.location.href = '/physical_ai_textbook/login';
  };

  const updateProfile = async (updates: Partial<UserProfile>) => {
    const token = localStorage.getItem('auth_token');
    if (!token) throw new Error('Not authenticated');

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${token}`,
      },
      body: JSON.stringify(updates),
    });

    const data = await response.json();
    if (!response.ok) throw new Error(data.error?.message || 'Profile update failed');

    localStorage.setItem('profile', JSON.stringify(data.profile));
    setProfile(data.profile);
  };

  const refreshProfile = async () => {
    const token = localStorage.getItem('auth_token');
    if (!token) return;

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      method: 'GET',
      headers: { Authorization: `Bearer ${token}` },
    });

    if (response.ok) {
      const data = await response.json();
      localStorage.setItem('profile', JSON.stringify(data.profile));
      setProfile(data.profile);
    }
  };

  return (
    <AuthContext.Provider value={{ user, profile, isAuthenticated: !!user, isLoading, login, signup, logout, updateProfile, refreshProfile }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = React.useContext(AuthContext);
  if (context === undefined) throw new Error('useAuth must be used within AuthProvider');
  return context;
};