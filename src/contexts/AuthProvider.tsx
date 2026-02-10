import React, { createContext, useState, useEffect, ReactNode, useContext } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface User { id: string; email: string; created_at: string; last_login: string | null; }
export interface UserProfile { python_level: string; ros2_level: string; gpu_available: string; hardware_tier: string; primary_goal: string; language_preference: string; }

export interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, profile: UserProfile) => Promise<void>;
  logout: () => void;
  updateProfile: (updates: Partial<UserProfile>) => Promise<void>;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    const storedUser = localStorage.getItem('user');
    const storedProfile = localStorage.getItem('profile');
    if (token && storedUser && storedProfile) {
      try {
        setUser(JSON.parse(storedUser));
        setProfile(JSON.parse(storedProfile));
      } catch (e) { localStorage.clear(); }
    }
    setIsLoading(false);
  }, []);

  // ✅ 1. implementation of updateProfile
  const updateProfile = async (updates: Partial<UserProfile>) => {
    const token = localStorage.getItem('auth_token');
    if (!token) throw new Error('Not authenticated');

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify(updates),
    });

    const data = await response.json();
    if (!response.ok) throw new Error(data.message || 'Update failed');

    const updatedProfile = { ...profile, ...data.profile } as UserProfile;
    setProfile(updatedProfile);
    localStorage.setItem('profile', JSON.stringify(updatedProfile));
  };

  const login = async (email: string, password: string) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.message || 'Login failed');
    localStorage.setItem('auth_token', data.session?.token || data.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile));
    setUser(data.user);
    setProfile(data.profile);
  };

  const signup = async (email: string, password: string, profileData: UserProfile) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password, profile: profileData }),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.message || 'Signup failed');
    localStorage.setItem('auth_token', data.session?.token || data.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile));
    setUser(data.user);
    setProfile(data.profile);
  };

  const logout = () => {
    localStorage.clear();
    setUser(null);
    setProfile(null);
    window.location.href = '/login';
  };

  return (
    // ✅ 2. updateProfile added to value object
    <AuthContext.Provider value={{ user, profile, isAuthenticated: !!user, isLoading, login, signup, logout, updateProfile }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};