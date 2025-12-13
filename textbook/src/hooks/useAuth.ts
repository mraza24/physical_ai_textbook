import { useContext } from 'react';
import { AuthContext, AuthContextType } from '../contexts/AuthProvider';

/**
 * Custom hook to access authentication context
 * Must be used within an AuthProvider
 */
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);

  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }

  return context;
};
