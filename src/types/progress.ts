/**
 * TypeScript types for chapter progress tracking API.
 * Matches backend Pydantic models in backend/app/models/chapter_progress.py
 */

/**
 * Progress status enum.
 */
export type ProgressStatus = 'started' | 'completed';

/**
 * Chapter progress response from API.
 */
export interface ChapterProgressResponse {
  /** Unique progress record ID */
  id: number;

  /** User ID (UUID) */
  user_id: string;

  /** Chapter ID (e.g., "module-1/intro-to-ros") */
  chapter_id: string;

  /** Progress status */
  status: ProgressStatus;

  /** Whether chapter is bookmarked */
  is_bookmarked: boolean;

  /** When progress was first recorded (ISO 8601) */
  started_at: string;

  /** When chapter was completed (ISO 8601, null if not completed) */
  completed_at: string | null;

  /** Last time this record was updated (ISO 8601) */
  updated_at: string;
}

/**
 * Request body for marking a chapter as started.
 */
export interface ChapterProgressMarkStarted {
  /** Chapter ID to mark as started */
  chapter_id: string;
}

/**
 * Request body for marking a chapter as completed.
 */
export interface ChapterProgressMarkCompleted {
  /** Chapter ID to mark as completed */
  chapter_id: string;
}

/**
 * Request body for toggling bookmark status.
 */
export interface ChapterProgressToggleBookmark {
  /** Chapter ID to toggle bookmark for */
  chapter_id: string;
}

/**
 * Query parameters for GET /api/v1/progress endpoint.
 */
export interface GetProgressQueryParams {
  /** Filter by status ('started' or 'completed') */
  status?: ProgressStatus | 'all';

  /** Only return bookmarked chapters */
  bookmarked_only?: boolean;
}
