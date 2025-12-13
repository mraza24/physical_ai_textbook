import React, { createContext, useState, useEffect, ReactNode } from 'react';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';

export interface User {
  id: string;
  email: string;
  created_at: string;
  last_login: string | null;
}

export interface UserProfile {
  python_level: string;
  ros2_level: string;
  gpu_available: string;
  hardware_tier: string;
  primary_goal: string;
  language_preference: string;
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
  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  // Load user from localStorage on mount
  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    const storedUser = localStorage.getItem('user');
    const storedProfile = localStorage.getItem('profile');

    if (token && storedUser && storedProfile) {
      try {
        setUser(JSON.parse(storedUser));
        setProfile(JSON.parse(storedProfile));
      } catch (error) {
        console.error('Failed to parse stored auth data:', error);
        localStorage.removeItem('auth_token');
        localStorage.removeItem('user');
        localStorage.removeItem('profile');
      }
    }

    setIsLoading(false);
  }, []);

  const signup = async (email: string, password: string, profileData: UserProfile) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password, profile: profileData }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error?.message || 'Signup failed');
    }

    const data = await response.json();

    // Store token and user data
    localStorage.setItem('auth_token', data.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile));

    setUser(data.user);
    setProfile(data.profile);
  };

  const login = async (email: string, password: string, rememberMe = false) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password, rememberMe }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error?.message || 'Login failed');
    }

    const data = await response.json();

    // Store token and user data
    localStorage.setItem('auth_token', data.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile));

    setUser(data.user);
    setProfile(data.profile);
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

    // Clear local state
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user');
    localStorage.removeItem('profile');
    setUser(null);
    setProfile(null);
  };

  const updateProfile = async (updates: Partial<UserProfile>) => {
    const token = localStorage.getItem('auth_token');

    if (!token) {
      throw new Error('Not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${token}`,
      },
      body: JSON.stringify(updates),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error?.message || 'Profile update failed');
    }

    const data = await response.json();

    // Update local state
    localStorage.setItem('profile', JSON.stringify(data.profile));
    setProfile(data.profile);
  };

  const refreshProfile = async () => {
    const token = localStorage.getItem('auth_token');

    if (!token) {
      throw new Error('Not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      method: 'GET',
      headers: {
        Authorization: `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error?.message || 'Failed to fetch profile');
    }

    const data = await response.json();

    // Update local state
    localStorage.setItem('profile', JSON.stringify(data.profile));
    setProfile(data.profile);
  };

  const value: AuthContextType = {
    user,
    profile,
    isAuthenticated: !!user,
    isLoading,
    login,
    signup,
    logout,
    updateProfile,
    refreshProfile,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};
