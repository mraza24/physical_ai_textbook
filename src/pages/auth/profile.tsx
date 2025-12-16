import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import { ProfileEditor } from '../../components/auth/ProfileEditor';
import { useAuth } from '../../hooks/useAuth';
import { useHistory } from '@docusaurus/router';

export default function ProfilePage(): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();
  const history = useHistory();

  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      // Redirect to signin if not authenticated
      history.push('/auth/signin');
    }
  }, [isAuthenticated, isLoading, history]);

  if (isLoading) {
    return (
      <Layout title="Profile">
        <main style={{ padding: '2rem', textAlign: 'center' }}>
          <p>Loading...</p>
        </main>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return (
      <Layout title="Profile">
        <main style={{ padding: '2rem', textAlign: 'center' }}>
          <p>Redirecting to sign in...</p>
        </main>
      </Layout>
    );
  }

  return (
    <Layout
      title="Edit Profile"
      description="Update your learning preferences"
    >
      <main>
        <ProfileEditor />
      </main>
    </Layout>
  );
}
